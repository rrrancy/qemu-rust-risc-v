// dram.rs - DRAM 模块
// 使用 mmap-rs 分配 2GB 宿主机内存模拟 DRAM

use byteorder::{ByteOrder, LittleEndian};
use mmap_rs::{MmapMut, MmapOptions};
use std::io::Read;

pub struct Dram {
    memory: MmapMut,
}

impl Dram {
    /// 创建 2GB DRAM
    pub fn new(size: usize) -> Result<Self, Box<dyn std::error::Error>> {
        // 使用 mmap 分配匿名内存，初始化为 0
        let options = MmapOptions::new(size)?;
        let memory = unsafe { options.map_mut() }
            .map_err(|e| format!("DRAM mmap 分配失败: {}", e))?;
        
        Ok(Self { memory })
    }

    /// 加载二进制数据到 DRAM 的指定偏移量
    pub fn load(&mut self, offset: usize, data: &[u8]) -> Result<(), &'static str> {
        if offset + data.len() > self.memory.len() {
            return Err("DRAM 加载：数据超出内存范围");
        }

        // 获取可变切片并复制数据
        let dest = unsafe { std::slice::from_raw_parts_mut(self.memory.as_mut_ptr(), self.memory.len()) };
        dest[offset..offset + data.len()].copy_from_slice(data);
        Ok(())
    }

    /// 从 reader 流式加载到整个 DRAM（避免一次性分配大内存）
    pub fn load_from_reader<R: Read>(
        &mut self,
        reader: &mut R,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let dest = unsafe { std::slice::from_raw_parts_mut(self.memory.as_mut_ptr(), self.memory.len()) };
        let mut offset = 0usize;
        while offset < dest.len() {
            let read_bytes = reader.read(&mut dest[offset..])?;
            if read_bytes == 0 {
                break;
            }
            offset += read_bytes;
        }

        if offset != dest.len() {
            return Err(format!(
                "DRAM 加载：读取字节数不足，期望 {} 实际 {}",
                dest.len(),
                offset
            )
            .into());
        }

        Ok(())
    }

    /// 读取 8 字节 (Little Endian)
    pub fn read64(&self, addr: u64) -> Result<u64, &'static str> {
        let addr = addr as usize;
        if addr + 8 > self.memory.len() {
            return Err("DRAM 读取64：地址超出范围");
        }
        let slice = unsafe { std::slice::from_raw_parts(self.memory.as_ptr(), self.memory.len()) };
        Ok(LittleEndian::read_u64(&slice[addr..addr + 8]))
    }

    /// 写入 8 字节 (Little Endian)
    pub fn write64(&mut self, addr: u64, value: u64) -> Result<(), &'static str> {
        let addr = addr as usize;
        if addr + 8 > self.memory.len() {
            return Err("DRAM 写入64：地址超出范围");
        }
        let slice = unsafe { std::slice::from_raw_parts_mut(self.memory.as_mut_ptr(), self.memory.len()) };
        LittleEndian::write_u64(&mut slice[addr..addr + 8], value);
        Ok(())
    }

    /// 读取 4 字节 (Little Endian)
    pub fn read32(&self, addr: u64) -> Result<u32, &'static str> {
        let addr = addr as usize;
        if addr + 4 > self.memory.len() {
            return Err("DRAM 读取32：地址超出范围");
        }
        let slice = unsafe { std::slice::from_raw_parts(self.memory.as_ptr(), self.memory.len()) };
        Ok(LittleEndian::read_u32(&slice[addr..addr + 4]))
    }

    /// 写入 4 字节 (Little Endian)
    pub fn write32(&mut self, addr: u64, value: u32) -> Result<(), &'static str> {
        let addr = addr as usize;
        if addr + 4 > self.memory.len() {
            return Err("DRAM 写入32：地址超出范围");
        }
        let slice = unsafe { std::slice::from_raw_parts_mut(self.memory.as_mut_ptr(), self.memory.len()) };
        LittleEndian::write_u32(&mut slice[addr..addr + 4], value);
        Ok(())
    }

    /// 读取 2 字节 (Little Endian)
    pub fn read16(&self, addr: u64) -> Result<u16, &'static str> {
        let addr = addr as usize;
        if addr + 2 > self.memory.len() {
            return Err("DRAM 读取16：地址超出范围");
        }
        let slice = unsafe { std::slice::from_raw_parts(self.memory.as_ptr(), self.memory.len()) };
        Ok(LittleEndian::read_u16(&slice[addr..addr + 2]))
    }

    /// 写入 2 字节 (Little Endian)
    pub fn write16(&mut self, addr: u64, value: u16) -> Result<(), &'static str> {
        let addr = addr as usize;
        if addr + 2 > self.memory.len() {
            return Err("DRAM 写入16：地址超出范围");
        }
        let slice = unsafe { std::slice::from_raw_parts_mut(self.memory.as_mut_ptr(), self.memory.len()) };
        LittleEndian::write_u16(&mut slice[addr..addr + 2], value);
        Ok(())
    }

    /// 读取 1 字节
    pub fn read8(&self, addr: u64) -> Result<u8, &'static str> {
        let addr = addr as usize;
        if addr >= self.memory.len() {
            return Err("DRAM 读取8：地址超出范围");
        }
        let slice = unsafe { std::slice::from_raw_parts(self.memory.as_ptr(), self.memory.len()) };
        Ok(slice[addr])
    }

    /// 写入 1 字节
    pub fn write8(&mut self, addr: u64, value: u8) -> Result<(), &'static str> {
        let addr = addr as usize;
        if addr >= self.memory.len() {
            return Err("DRAM 写入8：地址超出范围");
        }
        let slice = unsafe { std::slice::from_raw_parts_mut(self.memory.as_mut_ptr(), self.memory.len()) };
        slice[addr] = value;
        Ok(())
    }

    /// 获取 DRAM 大小
    pub fn size(&self) -> usize {
        self.memory.len()
    }
}
