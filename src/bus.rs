// bus.rs - 总线路由模块
// 所有地址均从 riscv64-virt.dts 解析而来，禁止硬编码！

use crate::dram::Dram;
use crate::flash::Flash;
use crate::mrom::Mrom;
use crate::plic::Plic;
use crate::uart::Uart;
use crate::virtio::VirtioBlock;
use crate::virtio_rng::VirtioRng;
use std::fs::File;
use std::sync::{Arc, Mutex};
use std::collections::VecDeque;

// ============ 从 DTS 提取的地址常量 ============
// memory@80000000: reg = <0x00 0x80000000 0x00 0x80000000>
pub const DRAM_BASE: u64 = 0x8000_0000;
pub const DRAM_SIZE: u64 = 0x8000_0000; // 2GB

// clint@2000000: reg = <0x00 0x2000000 0x00 0x10000>
pub const CLINT_BASE: u64 = 0x0200_0000;
pub const CLINT_SIZE: u64 = 0x0001_0000;

// plic@c000000: reg = <0x00 0xc000000 0x00 0x600000>
// 注意：DTS 中的 PLIC 大小是 0x600000，非标准大小！
pub const PLIC_BASE: u64 = 0x0c00_0000;
pub const PLIC_SIZE: u64 = 0x0060_0000;

// serial@10000000: reg = <0x00 0x10000000 0x00 0x100>
pub const UART_BASE: u64 = 0x1000_0000;
pub const UART_SIZE: u64 = 0x0000_0100;

// virtio_mmio@10001000 ... @10008000: reg = <0x00 0x1000X000 0x00 0x1000>
// 注意：QEMU virt 标准设备树将块设备放在 Device 1 (0x10002000)
pub const VIRTIO_BASE: u64 = 0x1000_1000;      // Device 0 起始地址
pub const VIRTIO_RNG_BASE: u64 = 0x1000_7000;  // Device 6 (RNG 设备位置)
pub const VIRTIO_BLK_BASE: u64 = 0x1000_2000;  // Device 1 (块设备实际位置)
pub const VIRTIO_SIZE: u64 = 0x0000_8000;      // 8 个设备，每个 0x1000

// rtc@101000: reg = <0x00 0x101000 0x00 0x1000>
pub const RTC_BASE: u64 = 0x0010_1000;
pub const RTC_SIZE: u64 = 0x0000_1000;

// test@100000: reg = <0x00 0x100000 0x00 0x1000>
pub const TEST_BASE: u64 = 0x0010_0000;
pub const TEST_SIZE: u64 = 0x0000_1000;

// fw-cfg@10100000: reg = <0x00 0x10100000 0x00 0x18>
pub const FW_CFG_BASE: u64 = 0x1010_0000;
pub const FW_CFG_SIZE: u64 = 0x0000_0018;

// flash@20000000: reg = <0x00 0x20000000 0x00 0x2000000 ...>
pub const FLASH_BASE: u64 = 0x2000_0000;
pub const FLASH_SIZE: u64 = 0x0200_0000;

// Flash Bank 1 (空设备占位，用于 U-Boot 探测)
pub const FLASH_BANK1_BASE: u64 = 0x2200_0000;
pub const FLASH_BANK1_SIZE: u64 = 0x0200_0000; // 32MB

// pci@30000000: reg = <0x00 0x30000000 0x00 0x10000000>
pub const PCI_BASE: u64 = 0x3000_0000;
pub const PCI_SIZE: u64 = 0x1000_0000;

// MROM (未在 DTS 中显式列出，但 QEMU virt 机器固定使用)
pub const MROM_BASE: u64 = 0x0000_1000;
pub const MROM_SIZE: u64 = 0x0000_f000;

// CLINT 寄存器偏移量
const CLINT_MSIP_OFFSET: u64 = 0x0000;       // msip 寄存器偏移（每个 hart 4 字节）
const CLINT_MTIME_OFFSET: u64 = 0xBFF8;      // mtime 寄存器偏移
const CLINT_MTIMECMP_OFFSET: u64 = 0x4000;   // mtimecmp 寄存器偏移

pub struct Bus {
    pub mrom: Mrom,
    pub dram: Dram,
    pub uart: Uart,
    pub flash: Flash,
    pub plic: Arc<Mutex<Plic>>,
    pub virtio_block: Option<VirtioBlock>,
    pub virtio_rng: VirtioRng,
    /// CLINT mtime 计数器（由 tick_timer 推进）
    mtime: u64,
    /// mtimecmp 寄存器（用于定时器中断）
    mtimecmp: u64,
    /// S-Mode Timer Compare (Sstc扩展)
    pub stimecmp: u64,
    /// CLINT MSIP 寄存器（软件中断）- 每个 hart 一个 bit
    msip: u64,
}

impl Bus {
    pub fn new(mrom: Mrom, dram: Dram, flash: Flash, drive_image: Option<File>) -> Self {
        let plic = Arc::new(Mutex::new(Plic::new()));
        
        // 初始化 VirtIO RNG 设备
        let mut virtio_rng = VirtioRng::new();
        let plic_for_rng = Arc::clone(&plic);
        virtio_rng.set_interrupt_callback(move || {
            plic_for_rng.lock().unwrap().set_pending(7); // RNG IRQ = 7
        });

        // 初始化 VirtIO Block 设备
        let virtio_block = drive_image.and_then(|file| {
            match VirtioBlock::new(file) {
                Ok(mut vb) => {
                    let plic_for_blk = Arc::clone(&plic);
                    vb.set_interrupt_callback(move || {
                        plic_for_blk.lock().unwrap().set_pending(2); // Block IRQ = 2 (Device 1 @ 0x10002000, 与 DTB 一致)
                    });
                    Some(vb)
                }
                Err(e) => {
                    eprintln!("[Bus] VirtIO Block 初始化失败: {}", e);
                    None
                }
            }
        });
        
        Self { 
            mrom, 
            dram,
            uart: Uart::new(),
            flash,
            plic,
            virtio_block,
            virtio_rng,
            mtime: 0,
            mtimecmp: 10000, // 默认 10000 ticks 后触发第一个中断（用于调试）
            stimecmp: u64::MAX, // 默认禁用S-Mode定时器中断
            msip: 0, // 初始化软件中断寄存器为 0
        }
    }

    /// 获取当前 mtime 值（由 tick_timer 推进）
    fn get_mtime(&self) -> u64 {
        self.mtime
    }

    /// 推进 CLINT mtime（每次增加 1）
    pub fn tick_timer(&mut self) {
        self.tick_timer_by(1);
    }

    /// 推进 CLINT mtime（按指定步进）
    pub fn tick_timer_by(&mut self, delta: u64) {
        self.mtime = self.mtime.wrapping_add(delta); 
    }
    
    /// 处理 VirtIO Block 队列请求
    fn process_virtio_block_queue(&mut self) -> Result<(), &'static str> {
        if let Some(ref mut vb) = self.virtio_block {
            vb.process_queue(&mut self.dram, DRAM_BASE)?;
        }
        Ok(())
    }
    
    /// 处理 VirtIO RNG 队列请求
    fn process_virtio_rng_queue(&mut self) -> Result<(), &'static str> {
        self.virtio_rng.process_queue(&mut self.dram, DRAM_BASE)
    }
    
    /// 更新 UART 中断状态到 PLIC
    /// 
    /// 在每次 UART 寄存器读写后调用，根据 UART 的中断条件
    /// 设置或清除 PLIC IRQ 10（UART 中断号）。
    /// 这实现了电平触发中断语义：当 UART 有活跃中断条件时保持 pending，
    /// 条件消失时清除 pending。
    fn update_uart_irq(&mut self) {
        const UART_IRQ: u32 = 10;
        if self.uart.has_pending_interrupt() {
            self.plic.lock().unwrap().set_pending(UART_IRQ);
        } else {
            self.plic.lock().unwrap().clear_pending(UART_IRQ);
        }
    }
    
    /// 公开的 UART 中断状态同步方法
    /// 模拟电平触发中断：在每次中断检查前调用，
    /// 确保 PLIC 反映 UART 的真实中断状态
    pub fn update_uart_irq_public(&mut self) {
        self.update_uart_irq();
    }
    
    /// 从总线读取 8 字节 (u64)
    pub fn read64(&mut self, addr: u64) -> Result<u64, &'static str> {
        match addr {
            // MROM 区域
            MROM_BASE..=0x0000_FFFF => self.mrom.read64(addr - MROM_BASE),

            // DRAM 区域
            a if a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE => {
                self.dram.read64(a - DRAM_BASE)
            },

            // CLINT 区域
            CLINT_BASE..=0x020F_FFFF => {
                let offset = addr - CLINT_BASE;
                match offset {
                    CLINT_MSIP_OFFSET => {
                        // 读取 msip（软件中断寄存器）
                        Ok(self.msip)
                    }
                    CLINT_MTIME_OFFSET => {
                        // 读取 mtime（64 位）
                        Ok(self.get_mtime())
                    }
                    CLINT_MTIMECMP_OFFSET => {
                        // 读取 mtimecmp（64 位）
                        Ok(self.mtimecmp)
                    }
                    _ => {
                        // 其他 CLINT 寄存器返回 0
                        Ok(0)
                    }
                }
            }

            // PLIC 区域
            PLIC_BASE..=0x0C5F_FFFF => {
                // 64位读取不常见，返回0
                Ok(0)
            }

            // UART 区域 - 64位访问映射到8位寄存器
            UART_BASE..=0x1000_00FF => {
                let offset = addr - UART_BASE;
                let result = self.uart.read8(offset);
                self.update_uart_irq();
                Ok(result? as u64)
            }

            // VirtIO Device 0 区域 (空设备: 0x10001000-0x10001FFF)
            VIRTIO_BASE..=0x1000_1FFF => Ok(0),
            
            // VirtIO Device 1 区域 (块设备: 0x10002000-0x10002FFF, 4KB)
            // 支持 64 位读取配置空间
            VIRTIO_BLK_BASE..=0x1000_2FFF => {
                let offset = addr - VIRTIO_BLK_BASE;
                if offset == 0x100 && self.virtio_block.is_some() {
                    let vb = self.virtio_block.as_mut().unwrap();
                    let capacity = vb.read32(0x100).unwrap_or(0) as u64
                        | ((vb.read32(0x104).unwrap_or(0) as u64) << 32);
                    Ok(capacity)
                } else {
                    Ok(0)
                }
            }
            
            // VirtIO Device 2-5 区域 (空设备: 0x10003000-0x10006FFF)
            0x1000_3000..=0x1000_6FFF => Ok(0),
            
            // VirtIO Device 6 区域 (RNG 设备: 0x10007000-0x10007FFF, 4KB)
            VIRTIO_RNG_BASE..=0x1000_7FFF => {
                // RNG 设备没有 64 位配置空间
                Ok(0)
            }
            
            // VirtIO Device 7 区域 (空设备: 0x10008000-0x10008FFF, 4KB)
            0x1000_8000..=0x1000_8FFF => Ok(0),

            // RTC 区域
            RTC_BASE..=0x0010_1FFF => {
                eprintln!("Unimplemented read64 at [RTC]: addr=0x{:016x}", addr);
                Ok(0)
            }

            // TEST (syscon) 区域
            TEST_BASE..=0x0010_0FFF => {
                eprintln!("Unimplemented read64 at [TEST]: addr=0x{:016x}", addr);
                Ok(0)
            }

            // FW-CFG 区域
            FW_CFG_BASE..=0x1010_0017 => {
                eprintln!("Unimplemented read64 at [FW-CFG]: addr=0x{:016x}", addr);
                Ok(0)
            }

            // Flash Bank 0 (CFI Parallel Flash)
            FLASH_BASE..=0x21FF_FFFF => {
                self.flash.read64(addr - FLASH_BASE)
            }

            // Flash Bank 1 (空设备 - U-Boot 探测用)
            FLASH_BANK1_BASE..=0x23FF_FFFF => {
                // 返回 0xFF 表示未连接设备（空 Flash）
                Ok(0xFFFF_FFFF_FFFF_FFFF)
            }

            // PCI ECAM 区域
            PCI_BASE..=0x3FFF_FFFF => Ok(0),

            _ => {
                eprintln!("Bus read64 ERROR: Address out of range: 0x{:016x}", addr);
                Err("总线读取：地址超出范围")
            }
        }
    }

    /// 向总线写入 8 字节 (u64)
    pub fn write64(&mut self, addr: u64, value: u64) -> Result<(), &'static str> {
        match addr {
            // DRAM 区域
            a if a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE => {
                self.dram.write64(a - DRAM_BASE, value)
            },

            // CLINT 区域
            CLINT_BASE..=0x020F_FFFF => {
                let offset = addr - CLINT_BASE;
                match offset {
                    CLINT_MSIP_OFFSET => {
                        // 写入 msip（软件中断寄存器）
                        // 只有 bit 0 有效（Hart 0 的软件中断）
                        self.msip = value & 1;
                        Ok(())
                    }
                    CLINT_MTIMECMP_OFFSET => {
                        // 写入 mtimecmp（64 位）
                        self.mtimecmp = value;
                        Ok(())
                    }
                    _ => {
                        // 其他 CLINT 寄存器忽略写入
                        Ok(())
                    }
                }
            }

            // PLIC 区域
            PLIC_BASE..=0x0C5F_FFFF => {
                // 64位写入不常见，忽略
                Ok(())
            }

            // UART 区域 - 64位访问映射到8位寄存器
            UART_BASE..=0x1000_00FF => {
                let offset = addr - UART_BASE;
                let result = self.uart.write8(offset, value as u8);
                self.update_uart_irq();
                result
            }

            // VirtIO Device 0 区域 (空设备: 0x10001000-0x10001FFF)
            VIRTIO_BASE..=0x1000_1FFF => Ok(()),
            
            // VirtIO Device 1 区域 (块设备: 0x10002000-0x10002FFF, 4KB)
            VIRTIO_BLK_BASE..=0x1000_2FFF => {
                // VirtIO MMIO 不支持 64 位写入
                Ok(())
            }
            
            // VirtIO Device 2-5 区域 (空设备: 0x10003000-0x10006FFF)
            0x1000_3000..=0x1000_6FFF => Ok(()),
            
            // VirtIO Device 6 区域 (RNG 设备: 0x10007000-0x10007FFF, 4KB)
            VIRTIO_RNG_BASE..=0x1000_7FFF => {
                // VirtIO MMIO 不支持 64 位写入
                Ok(())
            }
            
            // VirtIO Device 7 区域 (空设备: 0x10008000-0x10008FFF, 4KB)
            0x1000_8000..=0x1000_8FFF => Ok(()),

            // TEST (syscon) 区域 - 用于 poweroff/reboot
            TEST_BASE..=0x0010_0FFF => {
                eprintln!("Unimplemented write64 at [TEST]: addr=0x{:016x}, value=0x{:016x}", addr, value);
                Ok(())
            }

            // Flash Bank 0 (CFI 命令写入)
            FLASH_BASE..=0x21FF_FFFF => {
                self.flash.write64(addr - FLASH_BASE, value)
            }

            // Flash Bank 1 (空设备 - 忽略写入)
            FLASH_BANK1_BASE..=0x23FF_FFFF => {
                // 忽略写入到空 Flash
                Ok(())
            }

            // PCI ECAM 区域
            PCI_BASE..=0x3FFF_FFFF => Ok(()),

            _ => {
                eprintln!("Bus write64 ERROR: Address out of range or read-only: 0x{:016x}", addr);
                Err("总线写入：地址超出范围或只读区域")
            }
        }
    }

    /// 读取 4 字节 (u32)
    pub fn read32(&mut self, addr: u64) -> Result<u32, &'static str> {
        match addr {
            // MROM 区域
            MROM_BASE..=0x0000_FFFF => self.mrom.read32(addr - MROM_BASE),
            // DRAM 区域
            a if a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE => {
                self.dram.read32(a - DRAM_BASE)
            },
            // CLINT 区域
            CLINT_BASE..=0x020F_FFFF => {
                let offset = addr - CLINT_BASE;
                match offset {
                    CLINT_MSIP_OFFSET => {
                        // 读取 msip（软件中断寄存器）- 32位访问
                        Ok(self.msip as u32)
                    }
                    CLINT_MTIME_OFFSET => {
                        // 读取 mtime 低 32 位
                        Ok(self.get_mtime() as u32)
                    }
                    o if o == CLINT_MTIME_OFFSET + 4 => {
                        // 读取 mtime 高 32 位
                        Ok((self.get_mtime() >> 32) as u32)
                    }
                    CLINT_MTIMECMP_OFFSET => {
                        // 读取 mtimecmp 低 32 位
                        Ok(self.mtimecmp as u32)
                    }
                    o if o == CLINT_MTIMECMP_OFFSET + 4 => {
                        // 读取 mtimecmp 高 32 位
                        Ok((self.mtimecmp >> 32) as u32)
                    }
                    _ => Ok(0),
                }
            }
            // PLIC 区域
            PLIC_BASE..=0x0C5F_FFFF => {
                let offset = addr - PLIC_BASE;
                self.plic.lock().unwrap().read32(offset)
            }
            // UART 区域 - 32位访问映射到8位寄存器
            UART_BASE..=0x1000_00FF => {
                let offset = addr - UART_BASE;
                // UART 寄存器是 8 位的，32 位访问只读取低 8 位
                let result = self.uart.read8(offset);
                self.update_uart_irq();
                Ok(result? as u32)
            }
            // VirtIO Device 0 区域 (空设备: 0x10001000-0x10001FFF)
            VIRTIO_BASE..=0x1000_1FFF => Ok(0),
            
            // VirtIO Device 1 区域 (块设备: 0x10002000-0x10002FFF, 4KB)
            VIRTIO_BLK_BASE..=0x1000_2FFF => {
                if let Some(ref mut vb) = self.virtio_block {
                    let offset = addr - VIRTIO_BLK_BASE;
                    Ok(vb.read32(offset).unwrap_or(0))
                } else {
                    Ok(0)
                }
            }
            
            // VirtIO Device 2-5 区域 (空设备: 0x10003000-0x10006FFF)
            0x1000_3000..=0x1000_6FFF => Ok(0),
            
            // VirtIO Device 6 区域 (RNG 设备: 0x10007000-0x10007FFF, 4KB)
            VIRTIO_RNG_BASE..=0x1000_7FFF => {
                let offset = addr - VIRTIO_RNG_BASE;
                self.virtio_rng.read32(offset)
            }
            
            // VirtIO Device 7 区域 (空设备: 0x10008000-0x10008FFF, 4KB)
            0x1000_8000..=0x1000_8FFF => Ok(0),
            // Flash Bank 0
            FLASH_BASE..=0x21FF_FFFF => {
                self.flash.read32(addr - FLASH_BASE)
            }
            // Flash Bank 1 (空设备)
            FLASH_BANK1_BASE..=0x23FF_FFFF => {
                Ok(0xFFFF_FFFF)
            }
            // RTC 区域 (Goldfish RTC)
            RTC_BASE..=0x0010_1FFF => {
                let offset = addr - RTC_BASE;
                match offset {
                    0x00 => {
                        // TIME_LOW: 返回当前时间（秒）的低 32 位
                        let secs = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_secs();
                        Ok(secs as u32)
                    }
                    0x04 => {
                        // TIME_HIGH: 返回当前时间（秒）的高 32 位
                        let secs = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_secs();
                        Ok((secs >> 32) as u32)
                    }
                    _ => Ok(0),
                }
            }
            // FW-CFG 区域（目前不实现，只返回 0，避免内核 qemu_fw_cfg 驱动触发异常）
            FW_CFG_BASE..=0x1010_0017 => {
                eprintln!("Unimplemented read32 at [FW-CFG]: addr=0x{:016x}", addr);
                Ok(0)
            }
            // TEST/Syscon 区域
            TEST_BASE..=0x0010_0FFF => {
                Ok(0)
            }
            // PCI ECAM 区域
            PCI_BASE..=0x3FFF_FFFF => Ok(0),
            _ => {
                eprintln!("Bus read32 ERROR: Address out of range: 0x{:016x}", addr);
                Err("总线读取：地址超出范围")
            }
        }
    }

    /// 写入 4 字节 (u32)
    pub fn write32(&mut self, addr: u64, value: u32) -> Result<(), &'static str> {
        match addr {
            a if a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE => {
                self.dram.write32(a - DRAM_BASE, value)
            },
            // CLINT 区域
            CLINT_BASE..=0x020F_FFFF => {
                let offset = addr - CLINT_BASE;
                match offset {
                    CLINT_MSIP_OFFSET => {
                        // 写入 msip（软件中断寄存器）- 32位访问
                        // 只有 bit 0 有效（Hart 0 的软件中断）
                        self.msip = (value & 1) as u64;
                        Ok(())
                    }
                    CLINT_MTIMECMP_OFFSET => {
                        // 写入 mtimecmp 低 32 位
                        self.mtimecmp = (self.mtimecmp & 0xFFFF_FFFF_0000_0000) | (value as u64);
                        Ok(())
                    }
                    o if o == CLINT_MTIMECMP_OFFSET + 4 => {
                        // 写入 mtimecmp 高 32 位
                        self.mtimecmp = (self.mtimecmp & 0x0000_0000_FFFF_FFFF) | ((value as u64) << 32);
                        Ok(())
                    }
                    _ => Ok(()),
                }
            }
            // PLIC 区域
            PLIC_BASE..=0x0C5F_FFFF => {
                let offset = addr - PLIC_BASE;
                let result = self.plic.lock().unwrap().write32(offset, value);
                // PLIC 写入后（特别是 complete 操作），重新同步 UART 中断状态
                // 这确保电平触发的 UART 中断在 complete 后能立即重新 assert
                self.update_uart_irq();
                result
            }
            // UART 区域 - 32位访问映射到8位寄存器
            UART_BASE..=0x1000_00FF => {
                let offset = addr - UART_BASE;
                // UART 寄存器是 8 位的，32 位访问只写入低 8 位
                let result = self.uart.write8(offset, value as u8);
                self.update_uart_irq();
                result
            }
            // VirtIO Device 0 区域 (空设备: 0x10001000-0x10001FFF)
            VIRTIO_BASE..=0x1000_1FFF => Ok(()),
            
            // VirtIO Device 1 区域 (块设备: 0x10002000-0x10002FFF, 4KB)
            // VirtIO IRQ 映射: Device 1 -> IRQ 2
            VIRTIO_BLK_BASE..=0x1000_2FFF => {
                let offset = addr - VIRTIO_BLK_BASE;
                const VIRTIO_BLK_IRQ: u32 = 2;
                
                let is_queue_notify = offset == 0x50;
                let is_interrupt_ack = offset == 0x64;
                
                // 先写入寄存器
                if let Some(ref mut vb) = self.virtio_block {
                    vb.write32(offset, value)?;
                }
                
                // 如果是队列通知，处理请求并触发中断
                if is_queue_notify && value == 0 {
                    self.process_virtio_block_queue()?;
                }
                
                // InterruptACK 处理
                if is_interrupt_ack {
                    if let Some(ref mut vb) = self.virtio_block {
                        let irq_status = vb.read32(0x60).unwrap_or(0);
                        if irq_status == 0 {
                            self.plic.lock().unwrap().clear_pending(VIRTIO_BLK_IRQ);
                        } else {
                            self.plic.lock().unwrap().set_pending(VIRTIO_BLK_IRQ);
                        }
                    }
                }
                
                Ok(())
            }
            
            // VirtIO Device 2-5 区域 (空设备: 0x10003000-0x10006FFF)
            0x1000_3000..=0x1000_6FFF => Ok(()),
            
            // VirtIO Device 6 区域 (RNG 设备: 0x10007000-0x10007FFF, 4KB)
            // VirtIO IRQ 映射: Device 6 -> IRQ 7
            VIRTIO_RNG_BASE..=0x1000_7FFF => {
                let offset = addr - VIRTIO_RNG_BASE;
                const VIRTIO_RNG_IRQ: u32 = 7;
                
                let is_queue_notify = offset == 0x50;
                let is_interrupt_ack = offset == 0x64;
                
                // 先写入寄存器
                self.virtio_rng.write32(offset, value)?;
                
                // 如果是队列通知，处理请求并触发中断
                if is_queue_notify && value == 0 {
                    self.process_virtio_rng_queue()?;
                }
                
                // InterruptACK 处理
                if is_interrupt_ack {
                    let irq_status = self.virtio_rng.read32(0x60).unwrap_or(0);
                    if irq_status == 0 {
                        self.plic.lock().unwrap().clear_pending(VIRTIO_RNG_IRQ);
                    } else {
                        self.plic.lock().unwrap().set_pending(VIRTIO_RNG_IRQ);
                    }
                }
                
                Ok(())
            }
            
            // VirtIO Device 7 区域 (空设备: 0x10008000-0x10008FFF, 4KB)
            0x1000_8000..=0x1000_8FFF => Ok(()),
            // Flash Bank 0
            FLASH_BASE..=0x21FF_FFFF => {
                self.flash.write32(addr - FLASH_BASE, value)
            }
            // Flash Bank 1 (空设备 - 忽略写入)
            FLASH_BANK1_BASE..=0x23FF_FFFF => {
                Ok(())
            }
            // FW-CFG 区域（目前不实现，忽略写入）
            FW_CFG_BASE..=0x1010_0017 => {
                eprintln!(
                    "Unimplemented write32 at [FW-CFG]: addr=0x{:016x}, value=0x{:08x}",
                    addr,
                    value
                );
                Ok(())
            }
            // PCI ECAM 区域
            PCI_BASE..=0x3FFF_FFFF => Ok(()),
            _ => {
                eprintln!("Bus write32 WARNING: Ignoring write to unknown address: 0x{:016x}", addr);
                Ok(())
            }
        }
    }

    /// 读取 2 字节 (u16)
    pub fn read16(&mut self, addr: u64) -> Result<u16, &'static str> {
        match addr {
            // MROM 区域
            MROM_BASE..=0x0000_FFFF => self.mrom.read16(addr - MROM_BASE),
            // DRAM 区域
            a if a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE => {
                self.dram.read16(a - DRAM_BASE)
            },
            // PLIC 区域
            PLIC_BASE..=0x0C5F_FFFF => {
                // 16位读取不常见，返回0
                Ok(0)
            }
            // VirtIO Device 0 区域 (空设备)
            VIRTIO_BASE..=0x1000_1FFF => Ok(0),
            // VirtIO Device 1 区域 (块设备)
            VIRTIO_BLK_BASE..=0x1000_2FFF => Ok(0),
            // VirtIO Device 2-5 区域 (空设备)
            0x1000_3000..=0x1000_6FFF => Ok(0),
            // VirtIO Device 6 区域 (RNG)
            VIRTIO_RNG_BASE..=0x1000_7FFF => Ok(0),
            // VirtIO Device 7 区域 (空设备)
            0x1000_8000..=0x1000_8FFF => Ok(0),
            // PCI ECAM 区域
            PCI_BASE..=0x3FFF_FFFF => Ok(0),
            // Flash Bank 0
            FLASH_BASE..=0x21FF_FFFF => {
                self.flash.read16(addr - FLASH_BASE)
            }
            // Flash Bank 1 (空设备)
            FLASH_BANK1_BASE..=0x23FF_FFFF => {
                Ok(0xFFFF)
            }
            // FW-CFG 区域（目前不实现，只返回 0）
            FW_CFG_BASE..=0x1010_0017 => {
                eprintln!("Unimplemented read16 at [FW-CFG]: addr=0x{:016x}", addr);
                Ok(0)
            }
            _ => {
                eprintln!("Bus read16 ERROR: Address out of range: 0x{:016x}", addr);
                Err("总线读取：地址超出范围")
            }
        }
    }

    /// 写入 2 字节 (u16)
    pub fn write16(&mut self, addr: u64, value: u16) -> Result<(), &'static str> {
        match addr {
            a if a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE => {
                self.dram.write16(a - DRAM_BASE, value)
            },
            // PLIC 区域
            PLIC_BASE..=0x0C5F_FFFF => Ok(()),
            // VirtIO Device 0 区域 (空设备)
            VIRTIO_BASE..=0x1000_1FFF => Ok(()),
            // VirtIO Device 1 区域 (块设备)
            VIRTIO_BLK_BASE..=0x1000_2FFF => Ok(()),
            // VirtIO Device 2-5 区域 (空设备)
            0x1000_3000..=0x1000_6FFF => Ok(()),
            // VirtIO Device 6 区域 (RNG)
            VIRTIO_RNG_BASE..=0x1000_7FFF => Ok(()),
            // VirtIO Device 7 区域 (空设备)
            0x1000_8000..=0x1000_8FFF => Ok(()),
            // PCI ECAM 区域
            PCI_BASE..=0x3FFF_FFFF => Ok(()),
            // Flash Bank 0
            FLASH_BASE..=0x21FF_FFFF => {
                self.flash.write16(addr - FLASH_BASE, value)
            }
            // Flash Bank 1 (空设备 - 忽略写入)
            FLASH_BANK1_BASE..=0x23FF_FFFF => {
                Ok(())
            }
            // FW-CFG 区域（目前不实现，忽略写入）
            FW_CFG_BASE..=0x1010_0017 => {
                eprintln!(
                    "Unimplemented write16 at [FW-CFG]: addr=0x{:016x}, value=0x{:04x}",
                    addr,
                    value
                );
                Ok(())
            }
            _ => {
                eprintln!("Bus write16 WARNING: Ignoring write to unknown address: 0x{:016x}", addr);
                Ok(())
            }
        }
    }

    /// 读取 1 字节 (u8)
    pub fn read8(&mut self, addr: u64) -> Result<u8, &'static str> {
        match addr {
            // MROM 区域
            MROM_BASE..=0x0000_FFFF => self.mrom.read8(addr - MROM_BASE),
            // DRAM 区域
            a if a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE => {
                self.dram.read8(a - DRAM_BASE)
            },
            // UART 区域
            UART_BASE..=0x1000_00FF => {
                let offset = addr - UART_BASE;
                let result = self.uart.read8(offset);
                self.update_uart_irq();
                result
            },
            // PLIC 区域
            PLIC_BASE..=0x0C5F_FFFF => {
                // 8位读取不常见，返回0
                Ok(0)
            }
            // VirtIO Device 0 区域 (空设备: 0x10001000-0x10001FFF)
            VIRTIO_BASE..=0x1000_1FFF => Ok(0),
            
            // VirtIO Device 1 区域 (块设备: 0x10002000-0x10002FFF, 4KB)
            VIRTIO_BLK_BASE..=0x1000_2FFF => {
                let offset = addr - VIRTIO_BLK_BASE;
                if let Some(ref mut vb) = self.virtio_block {
                    if offset >= 0x100 {
                        let reg_offset = (offset & !0x3) as u64;
                        let byte_index = (offset & 0x3) as usize;
                        let reg_value = vb.read32(reg_offset).unwrap_or(0);
                        Ok(((reg_value >> (byte_index * 8)) & 0xFF) as u8)
                    } else {
                        Ok(0)
                    }
                } else {
                    Ok(0)
                }
            }
            
            // VirtIO Device 2-5 区域 (空设备: 0x10003000-0x10006FFF)
            0x1000_3000..=0x1000_6FFF => Ok(0),
            
            // VirtIO Device 6 区域 (RNG 设备: 0x10007000-0x10007FFF, 4KB)
            VIRTIO_RNG_BASE..=0x1000_7FFF => {
                let offset = addr - VIRTIO_RNG_BASE;
                if offset >= 0x100 {
                    // 配置空间：从 32 位寄存器中提取字节
                    let reg_offset = (offset & !0x3) as u64;
                    let byte_index = (offset & 0x3) as usize;
                    let reg_value = self.virtio_rng.read32(reg_offset).unwrap_or(0);
                    Ok(((reg_value >> (byte_index * 8)) & 0xFF) as u8)
                } else {
                    Ok(0)
                }
            }
            
            // VirtIO Device 7 区域 (空设备: 0x10008000-0x10008FFF, 4KB)
            0x1000_8000..=0x1000_8FFF => Ok(0),
            // PCI ECAM 区域
            PCI_BASE..=0x3FFF_FFFF => Ok(0),
            // Flash Bank 0
            FLASH_BASE..=0x21FF_FFFF => {
                self.flash.read8(addr - FLASH_BASE)
            }
            // Flash Bank 1 (空设备)
            FLASH_BANK1_BASE..=0x23FF_FFFF => {
                Ok(0xFF)
            }
            // FW-CFG 区域（目前不实现，只返回 0）
            FW_CFG_BASE..=0x1010_0017 => {
                eprintln!("Unimplemented read8 at [FW-CFG]: addr=0x{:016x}", addr);
                Ok(0)
            }
            _ => {
                eprintln!("Bus read8 ERROR: Address out of range: 0x{:016x}", addr);
                Err("总线读取：地址超出范围")
            }
        }
    }

    /// 写入 1 字节 (u8)
    pub fn write8(&mut self, addr: u64, value: u8) -> Result<(), &'static str> {
        match addr {
            a if a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE => {
                self.dram.write8(a - DRAM_BASE, value)
            },
            // UART 区域
            UART_BASE..=0x1000_00FF => {
                let offset = addr - UART_BASE;
                let result = self.uart.write8(offset, value);
                self.update_uart_irq();
                result
            },
            // PLIC 区域
            PLIC_BASE..=0x0C5F_FFFF => Ok(()),
            
            // VirtIO Device 0 区域 (空设备: 0x10001000-0x10001FFF)
            VIRTIO_BASE..=0x1000_1FFF => Ok(()),
            
            // VirtIO Device 1 区域 (块设备: 0x10002000-0x10002FFF, 4KB)
            // 配置空间是只读的，忽略字节写入
            VIRTIO_BLK_BASE..=0x1000_2FFF => Ok(()),
            
            // VirtIO Device 2-5 区域 (空设备: 0x10003000-0x10006FFF)
            0x1000_3000..=0x1000_6FFF => Ok(()),
            
            // VirtIO Device 6 区域 (RNG 设备: 0x10007000-0x10007FFF, 4KB)
            // 配置空间是只读的，忽略字节写入
            VIRTIO_RNG_BASE..=0x1000_7FFF => Ok(()),
            
            // VirtIO Device 7 区域 (空设备: 0x10008000-0x10008FFF, 4KB)
            0x1000_8000..=0x1000_8FFF => Ok(()),
            
            // PCI ECAM 区域
            PCI_BASE..=0x3FFF_FFFF => Ok(()),
            // Flash Bank 0
            FLASH_BASE..=0x21FF_FFFF => {
                self.flash.write8(addr - FLASH_BASE, value)
            }
            // Flash Bank 1 (空设备 - 忽略写入)
            FLASH_BANK1_BASE..=0x23FF_FFFF => {
                Ok(())
            }
            // FW-CFG 区域（目前不实现，忽略写入）
            FW_CFG_BASE..=0x1010_0017 => {
                eprintln!(
                    "Unimplemented write8 at [FW-CFG]: addr=0x{:016x}, value=0x{:02x}",
                    addr,
                    value
                );
                Ok(())
            }
            _ => {
                eprintln!("Bus write8 WARNING: Ignoring write to unknown address: 0x{:016x}", addr);
                Ok(())
            }
        }
    }
    
    /// 获取 UART 输入缓冲区的引用（用于从标准输入读取）
    pub fn get_uart_input_buffer(&self) -> Arc<Mutex<VecDeque<u8>>> {
        Arc::clone(&self.uart.input_buffer)
    }
    
    // ============ 中断状态查询接口 ============
    
    /// 检查是否有待处理的外部中断 (PLIC)
    /// 
    /// # 参数
    /// - `context_id`: PLIC Context ID (0 = M-Mode, 1 = S-Mode)
    /// 
    /// # 返回
    /// 如果有优先级 > threshold 且已使能的待处理中断，返回 true
    pub fn has_external_interrupt(&self, context_id: usize) -> bool {
        self.plic.lock().unwrap().has_pending_interrupt(context_id)
    }
    
    /// 检查是否有待处理的定时器中断 (CLINT)
    /// 
    /// # 返回
    /// 如果 mtime >= mtimecmp，返回 true（表示定时器中断待处理）
    pub fn has_timer_interrupt(&self) -> bool {
        let mtime = self.get_mtime();
        mtime >= self.mtimecmp
    }
    
    /// 获取当前 mtime 值（公开方法）
    pub fn get_mtime_public(&self) -> u64 {
        self.get_mtime()
    }
    
    /// 获取 mtimecmp 值（公开方法）
    pub fn get_mtimecmp(&self) -> u64 {
        self.mtimecmp
    }
    
    /// 检查 S-Mode 定时器中断 (Sstc扩展)
    /// 只有当 stimecmp 被设置过（不为 MAX）且当前时间 >= 设定时间时，才触发
    pub fn has_s_timer_interrupt(&self) -> bool {
        self.stimecmp != u64::MAX && self.mtime >= self.stimecmp
    }
    
    /// 检查 CLINT MSIP（软件中断）是否待处理
    /// 
    /// # 参数
    /// - `hart_id`: Hart ID (目前只支持 Hart 0)
    /// 
    /// # 返回
    /// 如果该 Hart 的 MSIP 位被设置，返回 true
    pub fn has_software_interrupt(&self, hart_id: usize) -> bool {
        // 检查对应 hart 的 msip 位
        // 每个 hart 对应 msip 中的一个 bit
        (self.msip >> hart_id) & 1 != 0
    }
}
