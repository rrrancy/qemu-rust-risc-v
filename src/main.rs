// main.rs - RISC-V 64-bit 解释型模拟器主程序
// 对标 QEMU 10.2.0 (TCI 模式)

mod bus;
mod cpu;
mod csr;
mod dram;
mod flash;
mod mmu;
mod mrom;
mod plic;
mod trap;
mod uart;
mod virtio;
mod virtio_rng;

use bus::{Bus, DRAM_BASE, DRAM_SIZE, FLASH_SIZE, MROM_SIZE};
use cpu::Cpu;
use dram::Dram;
use flash::Flash;
use mrom::Mrom;
use log::{error, info, warn};
use std::fs::{self, File, OpenOptions};
use std::io::{self, BufReader, Read};
use std::thread;
use std::sync::Arc;


// 启动地址配置
// 注意：根据需求选择启动模式
// Mode 1: 完整启动（从 MROM/OpenSBI 开始）- 正确的启动方式
const BOOT_PC: u64 = 0x0000_1000;
const SKIP_MROM: bool = false;

// Mode 2: 跳过 MROM，直接从内核入口启动（用于 Diff Test）
// const BOOT_PC: u64 = 0x8000_0000;
// const SKIP_MROM: bool = true;

const DTB_ADDR: u64 = 0x8700_0000; // DTB 加载地址 (物理地址)
const TIMER_TICK_INTERVAL: u64 = 10000; // 每 1000 条指令推进一次 mtime（更慢）
const TIMER_TICK_STEP: u64 = 1000; // 步进量，配合 timebase-frequency=10MHz
// 效果：每 1000 条指令 = 1ms 模拟时间

// ========== 全局调试开关 ==========
// 设置为 false 可以关闭所有额外的调试输出，只保留 MD5 和基本 UART 输出
pub const DEBUG_VERBOSE: bool = false;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 初始化日志 (通过 RUST_LOG 环境变量控制)
    env_logger::init();

    info!("========================================");
    info!("RISC-V 64-bit 解释型模拟器 (对标 QEMU 10.2.0 TCI)");
    info!("========================================");
    
    // 注意：终端保持默认的行缓冲模式
    // 用户需要按 Enter 才能发送输入到模拟器
    
    // ============ 1. 初始化 MROM（可选）============
    let mrom = if !SKIP_MROM {
        info!("[1/6] 初始化 MROM (Mask ROM)...");
        let mut m = Mrom::new(MROM_SIZE as usize)?;
        info!("      MROM 基地址: 0x{:08X}, 大小: 0x{:08X} ({} KB)",
             0x1000, MROM_SIZE, MROM_SIZE / 1024);

        // 加载 mrom.bin
        let mrom_data = fs::read("mrom.bin")?;
        info!("      加载 mrom.bin: {} 字节 ({:.2} KB)", mrom_data.len(), mrom_data.len() as f64 / 1024.0);
        m.load(0, &mrom_data)?;
        info!("      已加载到 MROM 偏移 0x00000000 (物理地址 0x00001000)");
        m
    } else {
        info!("[1/6] 跳过 MROM 初始化 (直接从内核启动模式)");
        Mrom::new(MROM_SIZE as usize)?
    };

    // ============ 2. 初始化 2GB DRAM ============
    info!("[2/6] 初始化 2GB DRAM (使用 mmap)...");
    let mut dram = Dram::new(DRAM_SIZE as usize)?;
    info!("      DRAM 基地址: 0x{:08X}, 大小: 0x{:08X} ({} MB)",
         DRAM_BASE, DRAM_SIZE, DRAM_SIZE / 1024 / 1024);

    // ============ 3. 以流式方式加载 dram.bin ============
    info!("[3/6] 加载 dram.bin (2GB)...");
    let dram_file = File::open("dram.bin")?;
    let dram_size = dram_file.metadata()?.len();
    if dram_size != DRAM_SIZE {
        return Err(format!(
            "dram.bin 大小不匹配，期望 {} 字节，实际 {} 字节",
            DRAM_SIZE, dram_size
        )
        .into());
    }
    info!("      dram.bin 大小: {} 字节 ({:.2} MB)", dram_size, dram_size as f64 / 1024.0 / 1024.0);

    let mut dram_reader = BufReader::new(dram_file);
    dram.load_from_reader(&mut dram_reader)?;
    info!("      已加载到 DRAM 偏移 0x00000000 (物理地址 0x{:08X})", DRAM_BASE);


    // ============ 4. 初始化 32 MiB CFI Flash ============
    info!("[4/8] 初始化 32 MiB CFI Parallel Flash...");
    let flash = Flash::new(FLASH_SIZE as usize);
    info!("      Flash 基地址: 0x{:08X}, 大小: 0x{:08X} ({} MB)",
         0x20000000, FLASH_SIZE, FLASH_SIZE / 1024 / 1024);
    info!("      CFI 协议: Intel/AMD 兼容，支持 CFI Query (0x98) 和 Read ID (0x90)");
    
    // ============ 5. 打开 VirtIO Block 设备 (drive.img) ============
    info!("[5/8] 打开 VirtIO Block 设备...");
    let drive_image = match OpenOptions::new()
        .read(true)
        .write(true)
        .open("drive.img")
    {
        Ok(file) => {
            let size = file.metadata()?.len();
            // 关键检查：文件大小为 0 时发出警告
            if size == 0 {
                warn!("      警告: drive.img 文件大小为 0！");
                warn!("      VirtIO 设备可能无法正常工作，请确认镜像文件是否正确生成。");
            }
            info!("      磁盘镜像: drive.img (读写模式)");
            info!("      大小: {} 字节 ({:.2} GB)", size, size as f64 / 1024.0 / 1024.0 / 1024.0);
            Some(file)
        }
        Err(e) => {
            warn!("      警告: 无法以读写模式打开 drive.img: {}", e);
            info!("      VirtIO Block 设备将不可用");
            None
        }
    };
    
    // ============ 6. 加载 DTB (Device Tree Blob) ============
    
    info!("[6/8] 加载 riscv64-virt.dtb (设备树)...");
    let dtb = fs::read("riscv64-virt.dtb")?;
    info!("      DTB 大小: {} 字节 ({:.2} KB)", dtb.len(), dtb.len() as f64 / 1024.0);
    
    // 将 DTB 加载到物理地址 0x8700_0000 (相对 DRAM 基地址的偏移)
    let dtb_offset = (DTB_ADDR - DRAM_BASE) as usize;
    dram.load(dtb_offset, &dtb)?;
    info!("      已加载到 DRAM 偏移 0x{:08X} (物理地址 0x{:08X})", dtb_offset, DTB_ADDR);
    

    // ============ 7. 初始化 CPU 和总线 ============
    info!("[7/8] 初始化 CPU 和总线...");
    let bus = Bus::new(mrom, dram, flash, drive_image);
    
    // 启动 UART 输入线程
    let uart_input = bus.get_uart_input_buffer();
    let plic_for_uart = Arc::clone(&bus.plic);
    thread::spawn(move || {
        let stdin = io::stdin();
        let mut buffer = [0u8; 1];
        loop {
            if let Ok(_) = stdin.lock().read_exact(&mut buffer) {
                let mut uart_buf = uart_input.lock().unwrap();
                uart_buf.push_back(buffer[0]);
                // UART 中断 ID 通常是 10
                plic_for_uart.lock().unwrap().set_pending(10);
            }
        }
    });
    
    let mut cpu = Cpu::new(bus);

    // 设置初始状态 (必须与 QEMU 初始状态一致!)
    cpu.pc = BOOT_PC;
    
    // ⚠️ 关键修复：根据 RISC-V 启动协议，必须设置以下寄存器
    // OpenSBI 和 Linux 内核都依赖这些参数！
    cpu.write_reg(10, 0);         // a0 (x10) = Hart ID = 0
    cpu.write_reg(11, DTB_ADDR);  // a1 (x11) = DTB 物理地址 = 0x8700_0000
    
    info!("      PC = 0x{:016X}", cpu.pc);
    info!("      a0 (Hart ID) = 0x{:016X}", cpu.read_reg(10));
    info!("      a1 (DTB Addr) = 0x{:016X}", cpu.read_reg(11));
    info!("      其他通用寄存器 = 0x0000000000000000");
    info!("      特权模式: Machine Mode");

    // ============ 8. 开始执行 ============
    info!("[8/8] 开始执行指令...");

    let mut inst_count = 0u64;
    
    loop {
        // 执行一条指令
        match cpu.step() {
            Ok(_) => {
                inst_count += 1;
                
                // 定期推进 CLINT mtime，确保定时器中断可触发
                if inst_count % TIMER_TICK_INTERVAL == 0 {
                    cpu.update_timer_by(TIMER_TICK_STEP);
                }

                // 限制执行步数
                if inst_count > 100000000000 {
                    info!("已执行 {} 条指令，停止模拟。", inst_count);
                    break;
                }
            }
            Err(_trap) => {
                error!("严重错误：step() 返回了未处理的 trap，这不应该发生！");
                break;
            }
        }
    }
    
    info!("模拟器已退出。");
    Ok(())
}

