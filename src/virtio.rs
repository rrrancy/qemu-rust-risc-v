// virtio.rs - VirtIO MMIO Block 设备实现
// 基于 VirtIO 1.0 Legacy Mode (MMIO)
// 支持 Scatter-Gather List (多描述符链)

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::sync::{Arc, Mutex};

// VirtIO MMIO 寄存器偏移量 (Legacy Interface)
const VIRTIO_MMIO_MAGIC_VALUE: u64 = 0x000;       // 0x74726976 ('virt')
const VIRTIO_MMIO_VERSION: u64 = 0x004;           // 0x1 (Legacy)
const VIRTIO_MMIO_DEVICE_ID: u64 = 0x008;         // 0x2 (Block device)
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

// VirtIO Block 配置空间 (从 0x100 开始)
const VIRTIO_BLK_CFG_CAPACITY: u64 = 0x100;       // 总扇区数 (64-bit)
const VIRTIO_BLK_CFG_SIZE_MAX: u64 = 0x108;       // 最大段大小 (32-bit)
const VIRTIO_BLK_CFG_SEG_MAX: u64 = 0x10c;        // 最大段数 (32-bit)
const VIRTIO_BLK_CFG_BLK_SIZE: u64 = 0x114;       // 块大小 (32-bit)

// VirtIO 状态位
const VIRTIO_STATUS_DRIVER_OK: u32 = 4;
const VIRTIO_STATUS_FAILED: u32 = 128;

// VirtIO Block 功能位
const VIRTIO_BLK_F_SIZE_MAX: u64 = 1 << 1;   // 最大段大小
const VIRTIO_BLK_F_SEG_MAX: u64 = 1 << 2;    // 最大段数
const VIRTIO_BLK_F_BLK_SIZE: u64 = 1 << 6;   // 块大小
const VIRTIO_BLK_F_FLUSH: u64 = 1 << 9;      // 支持 Flush/Barrier（U-Boot 要求）

// VirtIO Block 请求类型
const VIRTIO_BLK_T_IN: u32 = 0;       // 读
const VIRTIO_BLK_T_OUT: u32 = 1;      // 写
const VIRTIO_BLK_T_FLUSH: u32 = 4;    // 刷新缓存 (Flush/Barrier)
const VIRTIO_BLK_T_GET_ID: u32 = 8;   // 获取设备序列号

// VirtIO Descriptor 标志
const VIRTQ_DESC_F_NEXT: u16 = 1;     // 描述符有后继
const VIRTQ_DESC_F_WRITE: u16 = 2;    // 设备写入（Guest 读取）

// 扇区大小
const SECTOR_SIZE: u64 = 512;

/// VirtIO Descriptor（描述符，16 字节）
#[derive(Debug, Clone, Copy)]
struct VirtqDesc {
    addr: u64,      // Guest 物理地址
    len: u32,       // 长度
    flags: u16,     // 标志位
    next: u16,      // 下一个描述符索引
}

/// VirtIO Block 请求头（16 字节）
#[derive(Debug)]
struct VirtioBlkReqHeader {
    req_type: u32,  // 请求类型 (READ/WRITE)
    _reserved: u32, // 保留字段
    sector: u64,    // 起始扇区号
}

/// 数据缓冲区描述（用于 Scatter-Gather）
#[derive(Debug, Clone)]
struct DataBuffer {
    addr: u64,      // Guest 物理地址
    len: u32,       // 长度
}

/// VirtIO Block 设备
pub struct VirtioBlock {
    /// 磁盘镜像文件
    disk_image: Arc<Mutex<File>>,
    
    /// 设备状态寄存器
    status: u32,
    
    /// 队列选择器
    queue_sel: u32,

     /// 队列对齐 (Legacy Mode 特有，默认为 4096)
    queue_align: u32,
    
    /// 队列配置（支持多个队列，但我们只用队列0）
    queue_num: [u32; 1],           // 队列长度
    queue_pfn: [u32; 1],           // 队列物理页号
    queue_ready: [bool; 1],        // 队列就绪标志
    
    /// 驱动/设备功能协商
    driver_features: u64,
    device_features_sel: u32,
    driver_features_sel: u32,
    
    /// 中断状态 (bit 0: Used Buffer Notification)
    interrupt_status: u32,
    
    /// 磁盘容量（扇区数）
    capacity: u64,
    
    /// 中断回调（用于触发 PLIC 中断）
    interrupt_callback: Option<Box<dyn Fn() + Send>>,
}

impl VirtioBlock {
    /// 创建新的 VirtIO Block 设备
    pub fn new(disk_image: File) -> Result<Self, std::io::Error> {
        // 获取磁盘大小
        let capacity = disk_image.metadata()?.len() / SECTOR_SIZE;
        
        eprintln!("[VirtIO-Block] 初始化设备，容量: {} 扇区 ({} MB)",
                 capacity, capacity * SECTOR_SIZE / 1024 / 1024);
        
        Ok(Self {
            disk_image: Arc::new(Mutex::new(disk_image)),
            status: 0,
            queue_sel: 0,
            queue_num: [0],
            queue_pfn: [0],
            queue_ready: [false],
            driver_features: 0,
            device_features_sel: 0,
            driver_features_sel: 0,
            interrupt_status: 0,
            capacity,
            queue_align: 4096, // 默认初始化为 4096
            interrupt_callback: None,
        })
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
    
    /// 读取 32 位寄存器
    pub fn read32(&mut self, offset: u64) -> Result<u32, &'static str> {
        match offset {
            VIRTIO_MMIO_MAGIC_VALUE => Ok(0x74726976), // 'virt'
            VIRTIO_MMIO_VERSION => Ok(1), // Legacy
            VIRTIO_MMIO_DEVICE_ID => Ok(2), // Block device
            VIRTIO_MMIO_VENDOR_ID => Ok(0x554d4551), // 'QEMU'
            
            VIRTIO_MMIO_DEVICE_FEATURES => {
                if self.device_features_sel == 0 {
                    // 低 32 位：支持基本的块设备功能 + FLUSH
                    let features = VIRTIO_BLK_F_SIZE_MAX 
                                 | VIRTIO_BLK_F_SEG_MAX 
                                 | VIRTIO_BLK_F_BLK_SIZE
                                 | VIRTIO_BLK_F_FLUSH;
                    Ok(features as u32)
                } else {
                    Ok(0)
                }
            }
            VIRTIO_MMIO_QUEUE_ALIGN => Ok(self.queue_align), // 支持读取对齐值
            // 内核 WARN: total_sg > vq->split.vring.num 时会触发；journal 等大 I/O 可能超过 128 段
            VIRTIO_MMIO_QUEUE_NUM_MAX => Ok(256),
            
            VIRTIO_MMIO_QUEUE_PFN => {
                if self.queue_sel == 0 {
                    Ok(self.queue_pfn[0])
                } else {
                    Ok(0)
                }
            }
            
            VIRTIO_MMIO_QUEUE_READY => {
                if self.queue_sel == 0 {
                    Ok(if self.queue_ready[0] { 1 } else { 0 })
                } else {
                    Ok(0)
                }
            }
            
            VIRTIO_MMIO_INTERRUPT_STATUS => Ok(self.interrupt_status),
            VIRTIO_MMIO_STATUS => Ok(self.status),
            
            // Config Generation (0x0fc)
            0x0fc => Ok(0),
            
            // Block 配置空间 (0x100 开始)
            VIRTIO_BLK_CFG_CAPACITY => Ok(self.capacity as u32),
            0x104 => Ok((self.capacity >> 32) as u32),
            VIRTIO_BLK_CFG_SIZE_MAX => Ok(0x20000), // 128 KB
            VIRTIO_BLK_CFG_SEG_MAX => Ok(128),
            VIRTIO_BLK_CFG_BLK_SIZE => Ok(SECTOR_SIZE as u32),
            
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
                    self.queue_num[0] = value;
                }
                Ok(())
            }
            
            VIRTIO_MMIO_QUEUE_ALIGN => {
                self.queue_align = value; // 关键修复：保存驱动写入的对齐值
                Ok(())
            }
            
            VIRTIO_MMIO_QUEUE_PFN => {
                if self.queue_sel == 0 {
                    self.queue_pfn[0] = value;
                    self.queue_ready[0] = value != 0;
                }
                Ok(())
            }
            
            VIRTIO_MMIO_QUEUE_READY => {
                if self.queue_sel == 0 {
                    self.queue_ready[0] = value != 0;
                }
                Ok(())
            }
            
            VIRTIO_MMIO_QUEUE_NOTIFY => {
                Ok(())
            }
            
            VIRTIO_MMIO_INTERRUPT_ACK => {
                self.interrupt_status &= !value;
                Ok(())
            }
            
            VIRTIO_MMIO_STATUS => {
                if value == 0 {
                    // 设备重置
                    self.queue_num = [0];
                    self.queue_pfn = [0];
                    self.queue_ready = [false];
                    self.queue_sel = 0;
                    self.interrupt_status = 0;
                    self.driver_features = 0;
                    self.device_features_sel = 0;
                    self.driver_features_sel = 0;
                    self.queue_align = 4096; // 重置对齐值
                }
                
                self.status = value;
                
                // 只记录关键状态变化
                if value & VIRTIO_STATUS_DRIVER_OK != 0 {
                    eprintln!("[VirtIO-Block] 驱动就绪，设备可用");
                }
                if value & VIRTIO_STATUS_FAILED != 0 {
                    eprintln!("[VirtIO-Block] 设备初始化失败！");
                }
                Ok(())
            }
            
            _ => Ok(()),
        }
    }
    
    /// 处理队列请求（公开方法，供 Bus 调用）
    /// 支持 Scatter-Gather List（多描述符链）
    pub fn process_queue(&mut self, dram: &mut Dram, dram_base: u64) -> Result<(), &'static str> {
        let queue_pfn = self.queue_pfn[0];
        if queue_pfn == 0 {
            return Ok(());
        }
        
        let queue_addr = (queue_pfn as u64) << 12;
        let queue_num = self.queue_num[0] as usize;
        let align = self.queue_align as u64; // 使用动态对齐值
        
        // 计算 VirtQueue 布局
        let avail_ring_offset = queue_num * 16;
        let avail_flags_addr = queue_addr + avail_ring_offset as u64;
        let avail_idx_addr = queue_addr + avail_ring_offset as u64 + 2;

        // 关键修复：Used Ring 对齐计算
        // Used Ring starts at ceil((avail_ring_end)/align) * align
        let avail_ring_len = 4 + 2 * queue_num + 2;// flags(2) + idx(2) + ring(2*N) + used_event(2)
        let avail_ring_end = avail_ring_offset as u64 + avail_ring_len as u64;
        let used_ring_offset = (avail_ring_end + align - 1) / align * align;

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
            
            // 动态解析描述符链
            let mut desc_chain: Vec<VirtqDesc> = Vec::new();
            let mut current_idx = head_desc_idx;
            let mut chain_len = 0;
            const MAX_CHAIN_LEN: usize = 256;
            
            loop {
                if chain_len >= MAX_CHAIN_LEN {
                    return Err("描述符链过长");
                }
                
                let desc_addr = queue_addr + (current_idx * 16) as u64;
                let desc = read_descriptor(dram, desc_addr, dram_base)?;
                desc_chain.push(desc);
                chain_len += 1;
                
                if desc.flags & VIRTQ_DESC_F_NEXT == 0 {
                    break;
                }
                current_idx = desc.next as usize;
            }
            
            if desc_chain.len() < 2 {
                return Err("描述符链太短");
            }
            
            let header_desc = &desc_chain[0];
            let header = read_request_header(dram, header_desc.addr, dram_base)?;
            let status_desc = &desc_chain[desc_chain.len() - 1];
            
            let data_buffers: Vec<DataBuffer> = desc_chain[1..desc_chain.len()-1]
                .iter()
                .map(|d| DataBuffer { addr: d.addr, len: d.len })
                .collect();
            
            let total_data_len: u32 = data_buffers.iter().map(|b| b.len).sum();
            
            // 处理各类请求
            let (status, used_len) = match header.req_type {
                VIRTIO_BLK_T_IN => {
                    let s = self.handle_read_request_sg(dram, header.sector, &data_buffers, dram_base)?;
                    (s, total_data_len + 1)
                }
                VIRTIO_BLK_T_OUT => {
                    let s = self.handle_write_request_sg(dram, header.sector, &data_buffers, dram_base)?;
                    (s, 1)
                }
                VIRTIO_BLK_T_FLUSH => {
                    let s = self.handle_flush_request()?;
                    (s, 1)
                }
                VIRTIO_BLK_T_GET_ID => {
                    if !data_buffers.is_empty() {
                        let s = self.handle_get_id_request(dram, data_buffers[0].addr, data_buffers[0].len, dram_base)?;
                        (s, data_buffers[0].len + 1)
                    } else {
                        (2, 1) // VIRTIO_BLK_S_IOERR
                    }
                }
                _ => {
                    eprintln!("[VirtIO-Block] 未知请求类型: {}", header.req_type);
                    (2, 1)
                }
            };
            
            // 写入状态字节
            write_u8(dram, status_desc.addr, status, dram_base)?;
            
            // 写入 Used Ring Entry
            // 确保这里使用的是上面计算出的正确的 used_ring_offset
            let used_ring_entry_addr = queue_addr + used_ring_offset + 4 
                                      + (used_idx as u64 % queue_num as u64) * 8;
            write_u32(dram, used_ring_entry_addr, head_desc_idx as u32, dram_base)?;
            write_u32(dram, used_ring_entry_addr + 4, used_len, dram_base)?;
            
            used_idx = used_idx.wrapping_add(1);
        }
        
        // 内存屏障
        std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
        
        // 一次性更新 Used Ring idx
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
    
    /// 处理读取请求（支持 Scatter-Gather）
    fn handle_read_request_sg(
        &mut self,
        dram: &mut Dram,
        sector: u64,
        buffers: &[DataBuffer],
        dram_base: u64,
    ) -> Result<u8, &'static str> {
        let total_len: u64 = buffers.iter().map(|b| b.len as u64).sum();
        
        // 检查越界
        let sectors_needed = (total_len + SECTOR_SIZE - 1) / SECTOR_SIZE;
        if sector + sectors_needed > self.capacity {
            eprintln!("[VirtIO-Block] 读取越界：sector {} + {} > capacity {}",
                     sector, sectors_needed, self.capacity);
            return Ok(2); // VIRTIO_BLK_S_IOERR
        }
        
        // 从磁盘读取数据
        let offset = sector * SECTOR_SIZE;
        let mut disk_buffer = vec![0u8; total_len as usize];
        
        {
            let mut disk = self.disk_image.lock().unwrap();
            if let Err(e) = disk.seek(SeekFrom::Start(offset)) {
                eprintln!("[VirtIO-Block] Seek 失败: {}", e);
                return Ok(2);
            }
            if let Err(e) = disk.read_exact(&mut disk_buffer) {
                eprintln!("[VirtIO-Block] 读取失败: {}", e);
                return Ok(2);
            }
        }
        
        // 将数据分散写入各个缓冲区（Scatter）
        let mut disk_offset: usize = 0;
        for buf in buffers.iter() {
            let buf_len = buf.len as usize;
            let end = disk_offset + buf_len;
            
            if end > disk_buffer.len() {
                return Ok(2); // VIRTIO_BLK_S_IOERR
            }
            
            for j in 0..buf_len {
                write_u8(dram, buf.addr + j as u64, disk_buffer[disk_offset + j], dram_base)?;
            }
            
            disk_offset = end;
        }
        
        Ok(0) // VIRTIO_BLK_S_OK
    }
    
    /// 处理写入请求（支持 Scatter-Gather）
    fn handle_write_request_sg(
        &mut self,
        dram: &mut Dram,
        sector: u64,
        buffers: &[DataBuffer],
        dram_base: u64,
    ) -> Result<u8, &'static str> {
        let total_len: u64 = buffers.iter().map(|b| b.len as u64).sum();
        
        // 检查越界
        let sectors_needed = (total_len + SECTOR_SIZE - 1) / SECTOR_SIZE;
        if sector + sectors_needed > self.capacity {
            eprintln!("[VirtIO-Block] 写入越界：sector {} + {} > capacity {}",
                     sector, sectors_needed, self.capacity);
            return Ok(2); // VIRTIO_BLK_S_IOERR
        }
        
        // 从各个缓冲区收集数据（Gather）
        let mut disk_buffer = Vec::with_capacity(total_len as usize);
        for buf in buffers.iter() {
            let buf_len = buf.len as usize;
            for j in 0..buf_len {
                let byte = read_u8(dram, buf.addr + j as u64, dram_base)?;
                disk_buffer.push(byte);
            }
        }
        
        // 写入磁盘
        let offset = sector * SECTOR_SIZE;
        {
            let mut disk = self.disk_image.lock().unwrap();
            if let Err(e) = disk.seek(SeekFrom::Start(offset)) {
                eprintln!("[VirtIO-Block] Seek 失败: {}", e);
                return Ok(2);
            }
            if let Err(e) = std::io::Write::write_all(&mut *disk, &disk_buffer) {
                eprintln!("[VirtIO-Block] 写入失败: {}", e);
                return Ok(2);
            }
        }
        
        Ok(0) // VIRTIO_BLK_S_OK
    }
    
    /// 处理 Flush 请求 (Type 4)
    fn handle_flush_request(&mut self) -> Result<u8, &'static str> {
        let disk = self.disk_image.lock().unwrap();
        if let Err(e) = disk.sync_all() {
            eprintln!("[VirtIO-Block] Flush 失败: {}", e);
            return Ok(2);
        }
        Ok(0) // VIRTIO_BLK_S_OK
    }
    
    /// 处理 Get ID 请求 (Type 8)
    fn handle_get_id_request(
        &mut self,
        dram: &mut Dram,
        buffer_addr: u64,
        length: u32,
        dram_base: u64,
    ) -> Result<u8, &'static str> {
        const DEVICE_SERIAL: &[u8] = b"MYSIM-VIRTIO-BLK";
        
        let write_len = std::cmp::min(length as usize, DEVICE_SERIAL.len());
        
        for i in 0..write_len {
            write_u8(dram, buffer_addr + i as u64, DEVICE_SERIAL[i], dram_base)?;
        }
        for i in write_len..(length as usize) {
            write_u8(dram, buffer_addr + i as u64, 0, dram_base)?;
        }
        
        Ok(0)
    }
}

// ============ 辅助函数：读写 Guest 内存 ============

use crate::dram::Dram;

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

/// 读取请求头
fn read_request_header(dram: &Dram, addr: u64, dram_base: u64) -> Result<VirtioBlkReqHeader, &'static str> {
    let req_type = read_u32(dram, addr, dram_base)?;
    let reserved = read_u32(dram, addr + 4, dram_base)?;
    let sector = read_u64(dram, addr + 8, dram_base)?;
    
    Ok(VirtioBlkReqHeader {
        req_type,
        _reserved: reserved,
        sector,
    })
}
