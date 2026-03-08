// flash.rs - CFI (Common Flash Interface) Parallel Flash 设备模拟
// 实现 Intel/AMD 标准 CFI 协议，用于 U-Boot Flash 探测

use log::{debug, warn};

/// Flash 状态机
#[derive(Debug, Clone, Copy, PartialEq)]
enum FlashState {
    /// 正常读取数组模式（默认）
    ReadArray,
    /// CFI 查询模式（响应 0x98 命令）
    CfiQuery,
    /// 读取 ID 模式（响应 0x90 命令）
    ReadId,
}

/// CFI Parallel Flash 设备
pub struct Flash {
    /// Flash 存储数据（32 MiB）
    data: Vec<u8>,
    /// 当前状态机状态
    state: FlashState,
    /// 设备大小（字节）
    size: usize,
    /// 扇区大小（字节）
    sector_size: usize,
}

impl Flash {
    /// 创建一个 32 MiB 的 CFI Flash 设备
    pub fn new(size: usize) -> Self {
        Self {
            data: vec![0xFF; size], // Flash 默认全 0xFF（擦除状态）
            state: FlashState::ReadArray,
            size,
            sector_size: 256 * 1024, // 256 KiB 扇区
        }
    }

    /// 从文件加载 Flash 内容
    pub fn load(&mut self, offset: usize, data: &[u8]) -> Result<(), String> {
        if offset + data.len() > self.size {
            return Err(format!(
                "Flash 加载越界：offset={}, len={}, flash_size={}",
                offset,
                data.len(),
                self.size
            ));
        }
        self.data[offset..offset + data.len()].copy_from_slice(data);
        Ok(())
    }

    /// MMIO 读取（支持 1/2/4/8 字节）
    pub fn read(&mut self, offset: u64, size: u8) -> Result<u64, &'static str> {
        let addr = offset as usize;

        // 根据状态机返回不同内容
        match self.state {
            FlashState::ReadArray => {
                // 如果地址超出范围，返回 0xFF (模拟总线空闲/Flash未编程)
                if addr >= self.size {
                    return Ok(u64::MAX); // All 1s
                }
                self.read_data(addr, size)
            }
            FlashState::CfiQuery => {
                // CFI 查询模式：返回 CFI Query Table 数据
                self.read_cfi_query(addr, size)
            }
            FlashState::ReadId => {
                // 读取 ID 模式：返回厂商 ID 和设备 ID
                self.read_id(addr, size)
            }
        }
    }

    /// MMIO 写入（用于发送命令）
    pub fn write(&mut self, offset: u64, value: u64, size: u8) -> Result<(), &'static str> {
        let addr = offset as usize;

        // CFI 命令通常写入到特定地址（如 0x5555 或任意地址）
        // 这里简化实现：任意地址都可以接收命令
        
        // 提取命令字节（取最低字节）
        let cmd = (value & 0xFF) as u8;

        match cmd {
            0xFF | 0xF0 => {
                // Reset / Read Array 命令
                debug!("[Flash] 收到 Reset 命令 (0x{:02X})，切换到 ReadArray 模式", cmd);
                self.state = FlashState::ReadArray;
                Ok(())
            }
            0x98 => {
                // CFI Query 命令
                debug!("[Flash] 收到 CFI Query 命令 (0x98)，切换到 CfiQuery 模式");
                self.state = FlashState::CfiQuery;
                Ok(())
            }
            0x90 => {
                // Read ID 命令
                debug!("[Flash] 收到 Read ID 命令 (0x90)，切换到 ReadId 模式");
                self.state = FlashState::ReadId;
                Ok(())
            }
            0x10 | 0x40 => {
                // Program / Write 命令（暂不实现实际编程）
                warn!("[Flash] 收到 Program 命令 (0x{:02X})，已忽略（只读模拟）", cmd);
                Ok(())
            }
            0x20 => {
                // Erase Setup 命令（暂不实现）
                warn!("[Flash] 收到 Erase Setup 命令 (0x20)，已忽略");
                Ok(())
            }
            0xD0 => {
                // Erase Confirm 命令（暂不实现）
                warn!("[Flash] 收到 Erase Confirm 命令 (0xD0)，已忽略");
                Ok(())
            }
            _ => {
                // 未知命令，忽略
                debug!("[Flash] 收到未知命令 0x{:02X}，已忽略", cmd);
                Ok(())
            }
        }
    }

    /// 从 Flash 数据数组读取
    fn read_data(&self, addr: usize, size: u8) -> Result<u64, &'static str> {
        if addr + (size as usize) > self.size {
            return Err("Flash 读取越界");
        }

        let mut result = 0u64;
        for i in 0..(size as usize) {
            result |= (self.data[addr + i] as u64) << (i * 8);
        }
        Ok(result)
    }

    /// 获取 CFI 表中指定逻辑地址的字节值
    fn get_cfi_byte(&self, logical_addr: usize) -> u8 {
        match logical_addr {
            // === CFI Query Identification String ===
            0x10 => b'Q',  // 0x10: 'Q'
            0x11 => b'R',  // 0x11: 'R'
            0x12 => b'Y',  // 0x12: 'Y'
            
            // === System Interface String ===
            0x13 => 0x01,  // Primary Algorithm Command Set (Intel)
            0x14 => 0x00,  // 高字节
            0x15 => 0x00,  // Primary Extended Table Address (无)
            0x16 => 0x00,
            0x17 => 0x00,  // Alternate Algorithm Command Set (无)
            0x18 => 0x00,
            0x19 => 0x00,  // Alternate Extended Table Address (无)
            0x1A => 0x00,

            // === Vcc Logic Supply Requirements ===
            0x1B => 0x27,  // Vcc Min (2.7V)
            0x1C => 0x36,  // Vcc Max (3.6V)
            0x1D => 0x00,  // Vpp Min (不需要编程电压)
            0x1E => 0x00,  // Vpp Max
            
            // === Timing Information ===
            0x1F => 0x09,  // Typical Word Write Timeout (2^9 µs = 512 µs)
            0x20 => 0x00,  // Typical Buffer Write Timeout (不支持)
            0x21 => 0x0A,  // Typical Block Erase Timeout (2^10 ms = 1024 ms)
            0x22 => 0x00,  // Typical Chip Erase Timeout (不支持)
            0x23 => 0x04,  // Max Word Write Timeout (2^4 = 16x typical)
            0x24 => 0x00,  // Max Buffer Write Timeout
            0x25 => 0x04,  // Max Block Erase Timeout
            0x26 => 0x00,  // Max Chip Erase Timeout

            // === Device Geometry ===
            0x27 => 0x19,  // Device Size = 2^25 bytes = 32 MiB
            0x28 => 0x02,  // Flash Device Interface Code: x8/x16 (支持 32 位访问)
            0x29 => 0x00,
            0x2A => 0x00,  // Max Bytes in Multi-byte Write (不支持)
            0x2B => 0x00,
            
            // === Erase Block Regions (单一扇区大小) ===
            0x2C => 0x01,  // Number of Erase Block Regions = 1
            
            // Region 1 Info: 128 个 256 KiB 扇区
            0x2D => 0x7F,  // (扇区数 - 1) 低字节 = 127 (0x7F)
            0x2E => 0x00,  // 高字节
            0x2F => 0x00,  // 扇区大小 = 256 * 256 bytes = 256 KiB (低字节)
            0x30 => 0x04,  // 高字节

            // === Extended Query (可选，未实现) ===
            // 0x31+: 扩展命令集相关信息

            _ => {
                // 未定义区域返回 0x00
                debug!("[Flash CFI] 读取未定义的 CFI 逻辑地址 0x{:04X}", logical_addr);
                0x00
            }
        }
    }

    /// CFI Query Table 读取（返回 CFI 标准数据）
    /// 支持 32 位宽度 Flash（bank-width = 4），地址需要除以 4
    fn read_cfi_query(&self, addr: usize, size: u8) -> Result<u64, &'static str> {
        // 对于 32 位宽度的 Flash (bank-width = 4)，U-Boot 访问地址 = 逻辑地址 * 4
        // 因此需要将物理地址除以 4 来获得 CFI 表的逻辑地址
        let logical_addr = addr / 4;
         // 获取基础字节
         let val = self.get_cfi_byte(logical_addr) as u64;

         let result = if size == 4 {
            val | (val << 8) | (val << 16) | (val << 24)
        } else {
            val
        };
      
        Ok(result)
    }

    /// 获取 ID 表中指定逻辑地址的字节值
    fn get_id_byte(&self, logical_addr: usize) -> u8 {
        match logical_addr {
            0x00 => 0x89,  // Intel 厂商代码
            0x01 => 0x18,  // 设备 ID（32 Mbit = 32 MiB）
            0x02 => 0x00,  // 扩展设备信息
            _ => 0xFF,     // 其他地址返回 0xFF
        }
    }

    /// 读取厂商 ID 和设备 ID
    /// 支持 32 位宽度 Flash（bank-width = 4），地址需要除以 4
    fn read_id(&self, addr: usize, size: u8) -> Result<u64, &'static str> {
        // 对于 32 位宽度的 Flash (bank-width = 4)，地址需要除以 4
        let logical_addr = addr / 4;
        let val = self.get_id_byte(logical_addr) as u64;
        
        let result = if size == 4 {
            val | (val << 8) | (val << 16) | (val << 24)
        } else {
            val
        };
        

        Ok(result)
    }

    /// 读取 8 字节
    pub fn read64(&mut self, offset: u64) -> Result<u64, &'static str> {
        self.read(offset, 8)
    }

    /// 读取 4 字节
    pub fn read32(&mut self, offset: u64) -> Result<u32, &'static str> {
        self.read(offset, 4).map(|v| v as u32)
    }

    /// 读取 2 字节
    pub fn read16(&mut self, offset: u64) -> Result<u16, &'static str> {
        self.read(offset, 2).map(|v| v as u16)
    }

    /// 读取 1 字节
    pub fn read8(&mut self, offset: u64) -> Result<u8, &'static str> {
        self.read(offset, 1).map(|v| v as u8)
    }

    /// 写入 8 字节
    pub fn write64(&mut self, offset: u64, value: u64) -> Result<(), &'static str> {
        self.write(offset, value, 8)
    }

    /// 写入 4 字节
    pub fn write32(&mut self, offset: u64, value: u32) -> Result<(), &'static str> {
        self.write(offset, value as u64, 4)
    }

    /// 写入 2 字节
    pub fn write16(&mut self, offset: u64, value: u16) -> Result<(), &'static str> {
        self.write(offset, value as u64, 2)
    }

    /// 写入 1 字节
    pub fn write8(&mut self, offset: u64, value: u8) -> Result<(), &'static str> {
        self.write(offset, value as u64, 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cfi_query_x32() {
        let mut flash = Flash::new(32 * 1024 * 1024);
        
        // 发送 CFI Query 命令（对于 x32 模式，命令地址也需要乘以 4）
        flash.write(0x55 * 4, 0x98, 1).unwrap();
        
        // 读取 "QRY" 字符串（x32 模式：逻辑地址 * 4 = 物理地址）
        // 逻辑地址 0x10 -> 物理地址 0x40
        // 逻辑地址 0x11 -> 物理地址 0x44
        // 逻辑地址 0x12 -> 物理地址 0x48
        assert_eq!(flash.read32(0x10 * 4).unwrap() as u8, b'Q');
        assert_eq!(flash.read32(0x11 * 4).unwrap() as u8, b'R');
        assert_eq!(flash.read32(0x12 * 4).unwrap() as u8, b'Y');
        
        // 读取设备大小 (2^25 = 32 MiB)
        // 逻辑地址 0x27 -> 物理地址 0x9C
        assert_eq!(flash.read32(0x27 * 4).unwrap() as u8, 0x19);
        
        // Reset
        flash.write(0x00, 0xFF, 1).unwrap();
    }

    #[test]
    fn test_read_id_x32() {
        let mut flash = Flash::new(32 * 1024 * 1024);
        
        // 发送 Read ID 命令
        flash.write(0x00, 0x90, 1).unwrap();
        
        // 读取厂商 ID 和设备 ID（x32 模式）
        // 逻辑地址 0x00 -> 物理地址 0x00
        // 逻辑地址 0x01 -> 物理地址 0x04
        assert_eq!(flash.read32(0x00).unwrap() as u8, 0x89); // Intel
        assert_eq!(flash.read32(0x04).unwrap() as u8, 0x18); // 32 MiB
        
        // Reset
        flash.write(0x00, 0xFF, 1).unwrap();
    }

    #[test]
    fn test_read_array() {
        let mut flash = Flash::new(1024);
        
        // 加载数据
        let data = vec![0x12, 0x34, 0x56, 0x78];
        flash.load(0, &data).unwrap();
        
        // 读取数据
        assert_eq!(flash.read8(0).unwrap(), 0x12);
        assert_eq!(flash.read8(1).unwrap(), 0x34);
        assert_eq!(flash.read16(0).unwrap(), 0x3412); // 小端序
        assert_eq!(flash.read32(0).unwrap(), 0x78563412);
    }
}
