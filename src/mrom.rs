// mrom.rs - Mask ROM 模块
// 存储 OpenSBI 固件等启动代码

use byteorder::{ByteOrder, LittleEndian};

pub struct Mrom {
    memory: Vec<u8>,
}

impl Mrom {
    /// 创建 MROM (通常 60KB)
    pub fn new(size: usize) -> Result<Self, Box<dyn std::error::Error>> {
        let memory = vec![0; size];
        Ok(Self { memory })
    }

    /// 加载二进制数据到 MROM 的指定偏移量
    pub fn load(&mut self, offset: usize, data: &[u8]) -> Result<(), &'static str> {
        if offset + data.len() > self.memory.len() {
            return Err("MROM 加载：数据超出内存范围");
        }
        self.memory[offset..offset + data.len()].copy_from_slice(data);
        Ok(())
    }

    /// 读取 4 字节 (Little Endian)
    pub fn read32(&self, addr: u64) -> Result<u32, &'static str> {
        let addr = addr as usize;
        if addr + 4 > self.memory.len() {
            return Err("MROM 读取32：地址超出范围");
        }
        Ok(LittleEndian::read_u32(&self.memory[addr..addr + 4]))
    }

    /// 读取 8 字节 (Little Endian)
    pub fn read64(&self, addr: u64) -> Result<u64, &'static str> {
        let addr = addr as usize;
        if addr + 8 > self.memory.len() {
            return Err("MROM 读取64：地址超出范围");
        }
        Ok(LittleEndian::read_u64(&self.memory[addr..addr + 8]))
    }

    /// 读取 2 字节 (Little Endian)
    pub fn read16(&self, addr: u64) -> Result<u16, &'static str> {
        let addr = addr as usize;
        if addr + 2 > self.memory.len() {
            return Err("MROM 读取16：地址超出范围");
        }
        Ok(LittleEndian::read_u16(&self.memory[addr..addr + 2]))
    }

    /// 读取 1 字节
    pub fn read8(&self, addr: u64) -> Result<u8, &'static str> {
        let addr = addr as usize;
        if addr >= self.memory.len() {
            return Err("MROM 读取8：地址超出范围");
        }
        Ok(self.memory[addr])
    }

    /// 获取 MROM 大小
    pub fn size(&self) -> usize {
        self.memory.len()
    }
}
