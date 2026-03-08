// virtio_rng.rs - VirtIO MMIO RNG (随机数生成器) 设备实现
// 基于 VirtIO 1.0 Legacy Mode (MMIO)
// 用于满足 U-Boot 的 EFI_RNG_PROTOCOL 需求

use std::fs::File;
use std::io::Read;

use crate::dram::Dram;

// VirtIO MMIO 寄存器偏移量 (Legacy Interface)
const VIRTIO_MMIO_MAGIC_VALUE: u64 = 0x000;       // 0x74726976 ('virt')
const VIRTIO_MMIO_VERSION: u64 = 0x004;           // 0x1 (Legacy)
const VIRTIO_MMIO_DEVICE_ID: u64 = 0x008;         // 0x4 (RNG device)
const VIRTIO_MMIO_VENDOR_ID: u64 = 0x00c;         // 0x554d4551 ('QEMU')
const VIRTIO_MMIO_DEVICE_FEATURES: u64 = 0x010;   // 设备支持的功能
const VIRTIO_MMIO_DEVICE_FEATURES_SEL: u64 = 0x014;
const VIRTIO_MMIO_DRIVER_FEATURES: u64 = 0x020;   // 驱动选择的功能
const VIRTIO_MMIO_DRIVER_FEATURES_SEL: u64 = 0x024;
const VIRTIO_MMIO_GUEST_PAGE_SIZE: u64 = 0x028;   // Guest 页大小（Legacy）
const VIRTIO_MMIO_QUEUE_SEL: u64 = 0x030;         // 队列选择器
const VIRTIO_MMIO_QUEUE_NUM_MAX: u64 = 0x034;     // 队列最大长度
const VIRTIO_MMIO_QUEUE_NUM: u64 = 0x038;         // 队列实际长度
const VIRTIO_MMIO_QUEUE_ALIGN: u64 = 0x03c;       // 队列对齐（Legacy）
const VIRTIO_MMIO_QUEUE_PFN: u64 = 0x040;         // 队列物理页号（Legacy）
const VIRTIO_MMIO_QUEUE_READY: u64 = 0x044;       // 队列就绪标志
const VIRTIO_MMIO_QUEUE_NOTIFY: u64 = 0x050;      // 队列通知（触发处理）
const VIRTIO_MMIO_INTERRUPT_STATUS: u64 = 0x060;  // 中断状态
const VIRTIO_MMIO_INTERRUPT_ACK: u64 = 0x064;     // 中断确认
const VIRTIO_MMIO_STATUS: u64 = 0x070;            // 设备状态

// VirtIO 状态位
const VIRTIO_STATUS_DRIVER_OK: u32 = 4;
const VIRTIO_STATUS_FAILED: u32 = 128;

// VirtIO Descriptor 标志
const VIRTQ_DESC_F_NEXT: u16 = 1;     // 描述符有后继
const VIRTQ_DESC_F_WRITE: u16 = 2;    // 设备写入（Guest 读取）

/// VirtIO Descriptor（描述符，16 字节）
#[derive(Debug, Clone, Copy)]
struct VirtqDesc {
    addr: u64,      // Guest 物理地址
    len: u32,       // 长度
    flags: u16,     // 标志位
    next: u16,      // 下一个描述符索引
}

/// VirtIO RNG 设备
pub struct VirtioRng {
    /// 设备状态寄存器
    status: u32,
    
    /// 队列选择器
    queue_sel: u32,
    
    /// 队列配置（RNG 只有一个队列）
    queue_num: u32,           // 队列长度
    queue_pfn: u32,           // 队列物理页号
    queue_ready: bool,        // 队列就绪标志
    
    /// 驱动/设备功能协商
    driver_features: u64,
    device_features_sel: u32,
    driver_features_sel: u32,
    
    /// 中断状态 (bit 0: Used Buffer Notification)
    interrupt_status: u32,
    
    /// 随机数源（/dev/urandom）
    rng_source: Option<File>,
    
    /// 中断回调（用于触发 PLIC 中断）
    interrupt_callback: Option<Box<dyn Fn() + Send>>,
}

impl VirtioRng {
    /// 创建新的 VirtIO RNG 设备
    pub fn new() -> Self {
        // 尝试打开 /dev/urandom 作为随机数源
        let rng_source = match File::open("/dev/urandom") {
            Ok(f) => {
                eprintln!("[VirtIO-RNG] 初始化设备，使用 /dev/urandom 作为随机数源");
                Some(f)
            }
            Err(e) => {
                eprintln!("[VirtIO-RNG] 警告: 无法打开 /dev/urandom: {}", e);
                eprintln!("[VirtIO-RNG] 将使用伪随机数生成器");
                None
            }
        };
        
        Self {
            status: 0,
            queue_sel: 0,
            queue_num: 0,
            queue_pfn: 0,
            queue_ready: false,
            driver_features: 0,
            device_features_sel: 0,
            driver_features_sel: 0,
            interrupt_status: 0,
            rng_source,
            interrupt_callback: None,
        }
    }
    
    /// 设置中断回调
    #[allow(dead_code)]
    pub fn set_interrupt_callback<F>(&mut self, callback: F)
    where
        F: Fn() + Send + 'static,
    {
        self.interrupt_callback = Some(Box::new(callback));
    }
    
    /// 触发中断
    fn trigger_interrupt(&mut self) {
        self.interrupt_status |= 0x1; // Used Buffer Notification
        if let Some(ref callback) = self.interrupt_callback {
            callback();
        }
    }
    
    /// 生成随机字节
    fn generate_random_bytes(&mut self, count: usize) -> Vec<u8> {
        let mut buffer = vec![0u8; count];
        
        if let Some(ref mut rng) = self.rng_source {
            // 从 /dev/urandom 读取
            if rng.read_exact(&mut buffer).is_ok() {
                return buffer;
            }
        }
        
        // 如果 /dev/urandom 不可用，使用简单的伪随机数生成器
        // 注意：这不是密码学安全的，仅用于测试
        use std::time::{SystemTime, UNIX_EPOCH};
        let seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        
        let mut state = seed;
        for byte in buffer.iter_mut() {
            // 简单的 xorshift64
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            *byte = state as u8;
        }
        
        buffer
    }
    
    /// 读取 32 位寄存器
    pub fn read32(&mut self, offset: u64) -> Result<u32, &'static str> {
        match offset {
            VIRTIO_MMIO_MAGIC_VALUE => Ok(0x74726976), // 'virt'
            VIRTIO_MMIO_VERSION => Ok(1), // Legacy
            VIRTIO_MMIO_DEVICE_ID => Ok(4), // RNG device (VirtIO Device ID 4)
            VIRTIO_MMIO_VENDOR_ID => Ok(0x554d4551), // 'QEMU'
            
            VIRTIO_MMIO_DEVICE_FEATURES => {
                // RNG 设备不需要特殊功能位
                Ok(0)
            }
            
            VIRTIO_MMIO_QUEUE_NUM_MAX => Ok(64), // 队列最大长度
            
            VIRTIO_MMIO_QUEUE_PFN => {
                if self.queue_sel == 0 {
                    Ok(self.queue_pfn)
                } else {
                    Ok(0)
                }
            }
            
            VIRTIO_MMIO_QUEUE_READY => {
                if self.queue_sel == 0 {
                    Ok(if self.queue_ready { 1 } else { 0 })
                } else {
                    Ok(0)
                }
            }
            
            VIRTIO_MMIO_INTERRUPT_STATUS => Ok(self.interrupt_status),
            VIRTIO_MMIO_STATUS => Ok(self.status),
            
            // Config Generation (0x0fc)
            0x0fc => Ok(0),
            
            _ => Ok(0),
        }
    }
    
    /// 写入 32 位寄存器
    pub fn write32(&mut self, offset: u64, value: u32) -> Result<(), &'static str> {
        match offset {
            VIRTIO_MMIO_DEVICE_FEATURES_SEL => {
                self.device_features_sel = value;
                Ok(())
            }
            
            VIRTIO_MMIO_DRIVER_FEATURES => {
                if self.driver_features_sel == 0 {
                    self.driver_features = (self.driver_features & 0xFFFF_FFFF_0000_0000) | (value as u64);
                } else {
                    self.driver_features = (self.driver_features & 0x0000_0000_FFFF_FFFF) | ((value as u64) << 32);
                }
                Ok(())
            }
            
            VIRTIO_MMIO_DRIVER_FEATURES_SEL => {
                self.driver_features_sel = value;
                Ok(())
            }
            
            VIRTIO_MMIO_GUEST_PAGE_SIZE => Ok(()),
            
            VIRTIO_MMIO_QUEUE_SEL => {
                self.queue_sel = value;
                Ok(())
            }
            
            VIRTIO_MMIO_QUEUE_NUM => {
                if self.queue_sel == 0 {
                    self.queue_num = value;
                }
                Ok(())
            }
            
            VIRTIO_MMIO_QUEUE_ALIGN => Ok(()),
            
            VIRTIO_MMIO_QUEUE_PFN => {
                if self.queue_sel == 0 {
                    self.queue_pfn = value;
                    self.queue_ready = value != 0;
                }
                Ok(())
            }
            
            VIRTIO_MMIO_QUEUE_READY => {
                if self.queue_sel == 0 {
                    self.queue_ready = value != 0;
                }
                Ok(())
            }
            
            VIRTIO_MMIO_QUEUE_NOTIFY => Ok(()),
            
            VIRTIO_MMIO_INTERRUPT_ACK => {
                self.interrupt_status &= !value;
                Ok(())
            }
            
            VIRTIO_MMIO_STATUS => {
                if value == 0 {
                    // 设备重置
                    self.queue_num = 0;
                    self.queue_pfn = 0;
                    self.queue_ready = false;
                    self.queue_sel = 0;
                    self.interrupt_status = 0;
                    self.driver_features = 0;
                    self.device_features_sel = 0;
                    self.driver_features_sel = 0;
                }
                
                self.status = value;
                
                if value & VIRTIO_STATUS_DRIVER_OK != 0 {
                    eprintln!("[VirtIO-RNG] 驱动就绪，设备可用");
                }
                if value & VIRTIO_STATUS_FAILED != 0 {
                    eprintln!("[VirtIO-RNG] 设备初始化失败！");
                }
                Ok(())
            }
            
            _ => Ok(()),
        }
    }
    
    /// 处理队列请求
    /// RNG 设备很简单：Guest 提交一个缓冲区，Host 用随机数填充它
    pub fn process_queue(&mut self, dram: &mut Dram, dram_base: u64) -> Result<(), &'static str> {
        if self.queue_pfn == 0 {
            return Ok(());
        }
        
        let queue_addr = (self.queue_pfn as u64) << 12;
        let queue_num = self.queue_num as usize;
        
        if queue_num == 0 {
            return Ok(());
        }
        
        // 计算 VirtQueue 布局 (Legacy)
        let avail_ring_offset = queue_num * 16;
        let avail_flags_addr = queue_addr + avail_ring_offset as u64;
        let avail_idx_addr = queue_addr + avail_ring_offset as u64 + 2;
        let used_ring_offset = ((avail_ring_offset + 4 + 2 * queue_num + 2 + 4095) / 4096) * 4096;
        let used_idx_addr = queue_addr + used_ring_offset as u64 + 2;
        
        let avail_flags = read_u16(dram, avail_flags_addr, dram_base)?;
        let avail_idx = read_u16(dram, avail_idx_addr, dram_base)?;
        let mut used_idx = read_u16(dram, used_idx_addr, dram_base)?;
        
        // 如果没有待处理的请求，直接返回
        if avail_idx == used_idx {
            return Ok(());
        }
        
        let start_used_idx = used_idx;
        
        // 处理所有待处理的请求
        while avail_idx != used_idx {
            let avail_ring_entry_addr = queue_addr + avail_ring_offset as u64 + 4 
                                       + (used_idx as u64 % queue_num as u64) * 2;
            let head_desc_idx = read_u16(dram, avail_ring_entry_addr, dram_base)? as usize;
            
            // 解析描述符链并填充随机数
            let mut total_written: u32 = 0;
            let mut current_idx = head_desc_idx;
            let mut chain_len = 0;
            const MAX_CHAIN_LEN: usize = 64;
            
            loop {
                if chain_len >= MAX_CHAIN_LEN {
                    break;
                }
                
                let desc_addr = queue_addr + (current_idx * 16) as u64;
                let desc = read_descriptor(dram, desc_addr, dram_base)?;
                
                // 只处理设备可写的缓冲区 (VIRTQ_DESC_F_WRITE)
                if desc.flags & VIRTQ_DESC_F_WRITE != 0 {
                    // 生成随机数并写入缓冲区
                    let random_bytes = self.generate_random_bytes(desc.len as usize);
                    
                    for (i, byte) in random_bytes.iter().enumerate() {
                        write_u8(dram, desc.addr + i as u64, *byte, dram_base)?;
                    }
                    
                    total_written += desc.len;
                }
                
                chain_len += 1;
                
                if desc.flags & VIRTQ_DESC_F_NEXT == 0 {
                    break;
                }
                current_idx = desc.next as usize;
            }
            
            // 写入 Used Ring Entry
            let used_ring_entry_addr = queue_addr + used_ring_offset as u64 + 4 
                                      + (used_idx as u64 % queue_num as u64) * 8;
            write_u32(dram, used_ring_entry_addr, head_desc_idx as u32, dram_base)?;
            write_u32(dram, used_ring_entry_addr + 4, total_written, dram_base)?;
            
            used_idx = used_idx.wrapping_add(1);
        }
        
        // 内存屏障
        std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
        
        // 更新 Used Ring idx
        if used_idx != start_used_idx {
            write_u16(dram, used_idx_addr, used_idx, dram_base)?;
        }
        
        // 检查是否应该触发中断
        const VRING_AVAIL_F_NO_INTERRUPT: u16 = 1;
        if (avail_flags & VRING_AVAIL_F_NO_INTERRUPT) == 0 {
            self.trigger_interrupt();
        }
        
        Ok(())
    }
}

impl Default for VirtioRng {
    fn default() -> Self {
        Self::new()
    }
}

// ============ 辅助函数：读写 Guest 内存 ============

fn read_u8(dram: &Dram, addr: u64, dram_base: u64) -> Result<u8, &'static str> {
    dram.read8(addr - dram_base)
}

fn write_u8(dram: &mut Dram, addr: u64, value: u8, dram_base: u64) -> Result<(), &'static str> {
    dram.write8(addr - dram_base, value)
}

fn read_u16(dram: &Dram, addr: u64, dram_base: u64) -> Result<u16, &'static str> {
    dram.read16(addr - dram_base)
}

fn write_u16(dram: &mut Dram, addr: u64, value: u16, dram_base: u64) -> Result<(), &'static str> {
    dram.write16(addr - dram_base, value)
}

fn read_u32(dram: &Dram, addr: u64, dram_base: u64) -> Result<u32, &'static str> {
    dram.read32(addr - dram_base)
}

fn write_u32(dram: &mut Dram, addr: u64, value: u32, dram_base: u64) -> Result<(), &'static str> {
    dram.write32(addr - dram_base, value)
}

fn read_u64(dram: &Dram, addr: u64, dram_base: u64) -> Result<u64, &'static str> {
    dram.read64(addr - dram_base)
}

/// 读取描述符
fn read_descriptor(dram: &Dram, addr: u64, dram_base: u64) -> Result<VirtqDesc, &'static str> {
    let addr_val = read_u64(dram, addr, dram_base)?;
    let len = read_u32(dram, addr + 8, dram_base)?;
    let flags = read_u16(dram, addr + 12, dram_base)?;
    let next = read_u16(dram, addr + 14, dram_base)?;
    
    Ok(VirtqDesc {
        addr: addr_val,
        len,
        flags,
        next,
    })
}
