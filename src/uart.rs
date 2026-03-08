// uart.rs - 16550 UART 模拟
// 参考: http://byterunner.com/16550.html

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::io::{self, Write};

/// UART 寄存器偏移 (16550 标准)
pub const UART_RHR: u64 = 0x0; // Receiver Holding Register (read)
pub const UART_THR: u64 = 0x0; // Transmitter Holding Register (write)
pub const UART_IER: u64 = 0x1; // Interrupt Enable Register
pub const UART_ISR: u64 = 0x2; // Interrupt Status Register (read)
pub const UART_FCR: u64 = 0x2; // FIFO Control Register (write)
pub const UART_LCR: u64 = 0x3; // Line Control Register
pub const UART_MCR: u64 = 0x4; // Modem Control Register
pub const UART_LSR: u64 = 0x5; // Line Status Register
pub const UART_MSR: u64 = 0x6; // Modem Status Register
pub const UART_SCR: u64 = 0x7; // Scratch Register

/// LSR (Line Status Register) 位定义
pub const LSR_DR: u8 = 0x01;   // Data Ready
pub const LSR_OE: u8 = 0x02;   // Overrun Error
pub const LSR_PE: u8 = 0x04;   // Parity Error
pub const LSR_FE: u8 = 0x08;   // Framing Error
pub const LSR_BI: u8 = 0x10;   // Break Interrupt
pub const LSR_THRE: u8 = 0x20; // Transmitter Holding Register Empty (bit 5)
pub const LSR_TEMT: u8 = 0x40; // Transmitter Empty (bit 6)
pub const LSR_IDLE: u8 = 0x60; // 0x20 | 0x40 = THRE | TEMT (发送器完全空闲)

pub struct Uart {
    /// 寄存器存储 (简化实现，只存储关键寄存器)
    pub ier: u8,  // Interrupt Enable Register
    lcr: u8,  // Line Control Register
    mcr: u8,  // Modem Control Register
    scr: u8,  // Scratch Register
    /// 输入缓冲区（用于接收字符）
    pub input_buffer: Arc<Mutex<VecDeque<u8>>>,
    /// 输出缓冲区（用于检测 panic 等关键字）
    output_buffer: String,
    /// panic 检测标志
    pub panic_detected: bool,
    /// panic 检测后的指令计数
    pub panic_post_count: u64,
    /// THRE (Transmitter Holding Register Empty) 中断待处理标志
    /// 模拟 QEMU 的 thr_ipending 行为：
    /// - 设置时机：THR 写入后（发送完成，THR 变空）、ETBEI 从 0→1 使能时
    /// - 清除时机：ISR 读取识别到 THRE 中断时
    /// 这是 Linux 8250 串口驱动中断驱动发送的关键！
    thr_ipending: bool,
    /// THRE 中断触发计数（调试用）
    pub thre_interrupt_count: u64,
    /// THR 写入字符计数（调试用）
    pub thr_write_count: u64,
}

impl Uart {
    pub fn new() -> Self {
        Self {
            ier: 0,
            lcr: 0,
            mcr: 0,
            scr: 0,
            input_buffer: Arc::new(Mutex::new(VecDeque::new())),
            output_buffer: String::with_capacity(1024),
            panic_detected: false,
            panic_post_count: 0,
            thr_ipending: true,  // THR 初始为空，THRE 中断初始为待处理
            thre_interrupt_count: 0,
            thr_write_count: 0,
        }
    }
    
    /// 检查是否检测到 panic
    pub fn check_panic(&self) -> bool {
        self.panic_detected
    }
    
    /// 重置 panic 状态
    pub fn reset_panic(&mut self) {
        self.panic_detected = false;
        self.panic_post_count = 0;
    }
    
    /// 增加 panic 后计数
    pub fn inc_panic_post_count(&mut self) {
        if self.panic_detected {
            self.panic_post_count += 1;
        }
    }

    /// 读取 UART 寄存器 (8 位)
    /// 注意：需要 &mut self 因为读取 ISR 有副作用（清除 THRE 中断状态）
    pub fn read8(&mut self, offset: u64) -> Result<u8, &'static str> {
        match offset {
            UART_RHR => {
                // 接收缓冲区：从输入缓冲区读取一个字符
                let mut buffer = self.input_buffer.lock().unwrap();
                if let Some(ch) = buffer.pop_front() {
                    Ok(ch)
                } else {
                    Ok(0) // 没有输入时返回 0
                }
            }
            UART_IER => Ok(self.ier),
            UART_ISR => {
                // Interrupt Status Register (16550 标准)
                // Bit 0: 0 = 有中断待处理，1 = 无中断
                // Bits 3:1: 中断 ID (优先级从高到低)
                //   0b110 (6) = 接收器线状态错误
                //   0b100 (4) = 接收数据就绪
                //   0b010 (2) = 发送器保持寄存器空 (THRE)
                //   0b000 (0) = Modem 状态
                // Bits 7:6: FIFO 状态
                
                let buffer = self.input_buffer.lock().unwrap();
                let has_input = !buffer.is_empty();
                drop(buffer);
                
                if has_input && (self.ier & 0x01) != 0 {
                    // 接收数据就绪中断 (IER bit 0 = ERBFI)
                    // 优先级高于 THRE
                    Ok(0x04)
                } else if self.thr_ipending && (self.ier & 0x02) != 0 {
                    // 发送器保持寄存器空中断 (IER bit 1 = ETBEI)
                    // ⚠️ 关键：读取 ISR 识别到 THRE 中断时，清除 thr_ipending
                    // 这模拟了 16550 的行为：ISR 读取会重置 THRE 中断源
                    // 新的 THRE 中断只有在下次 THR 写入或 ETBEI 重新使能后才会触发
                    self.thr_ipending = false;
                    self.thre_interrupt_count += 1;
                    Ok(0x02)
                } else {
                    // 无中断待处理
                    Ok(0x01)
                }
            }
            UART_LCR => Ok(self.lcr),
            UART_MCR => Ok(self.mcr),
            UART_LSR => {
                // Line Status Register
                // Bit 0 (DR): Data Ready - 接收缓冲区有数据
                // Bit 5 (THRE): Transmitter Holding Register Empty
                // Bit 6 (TEMT): Transmitter Empty
                let buffer = self.input_buffer.lock().unwrap();
                let has_input = !buffer.is_empty();
                drop(buffer);
                
                let mut lsr = LSR_IDLE; // 0x60 = LSR_THRE | LSR_TEMT
                if has_input {
                    lsr |= LSR_DR; // 设置 Data Ready 位
                }
                Ok(lsr)
            }
            UART_MSR => {
                // Modem Status Register (暂时返回固定值)
                Ok(0x00)
            }
            UART_SCR => Ok(self.scr),
            _ => {
                // 未定义的寄存器返回 0
                Ok(0)
            }
        }
    }

    /// 写入 UART 寄存器 (8 位)
    pub fn write8(&mut self, offset: u64, value: u8) -> Result<(), &'static str> {
        match offset {
            UART_THR => {
                // 发送字符到标准输出
                let mut stdout = io::stdout();
                let buffer = [value];
                stdout.write_all(&buffer).unwrap(); 
                stdout.flush().unwrap();
                self.thr_write_count += 1;
                
                // 添加到输出缓冲区用于 panic 检测
                if value.is_ascii() && value != 0 {
                    self.output_buffer.push(value as char);
                    // 保持缓冲区在合理大小
                    if self.output_buffer.len() > 2048 {
                        self.output_buffer = self.output_buffer[1024..].to_string();
                    }
                    // 检测 panic 关键字（如需详细 panic 记录，可在此处恢复调试输出）
                    if !self.panic_detected && 
                       (self.output_buffer.contains("Kernel panic") || 
                        self.output_buffer.contains("kernel panic") ||
                        self.output_buffer.contains("not syncing")) {
                        self.panic_detected = true;
                    }
                }
                // ⚠️ 关键修复：THR 写入后立即完成发送（我们直接写stdout），
                // THR 再次变空，重新触发 THRE 中断待处理
                // 这样 Linux 8250 驱动可以在下一次中断中继续发送剩余数据
                self.thr_ipending = true;
                Ok(())
            }
            UART_IER => {
                let old_ier = self.ier;
                self.ier = value;
                // ⚠️ 关键修复：当 ETBEI (bit 1) 从 0 变为 1 时，
                // 如果 THR 为空（在我们的模拟器中总是空的），触发 THRE 中断
                // 这是 Linux serial8250_start_tx() 启动发送流程的入口！
                if (old_ier & 0x02) == 0 && (value & 0x02) != 0 {
                    self.thr_ipending = true;
                }
                Ok(())
            }
            UART_FCR => {
                // FIFO Control Register (暂不实现 FIFO)
                Ok(())
            }
            UART_LCR => {
                self.lcr = value;
                Ok(())
            }
            UART_MCR => {
                self.mcr = value;
                Ok(())
            }
            UART_SCR => {
                self.scr = value;
                Ok(())
            }
            _ => {
                // 忽略未定义的寄存器写入
                Ok(())
            }
        }
    }
    
    /// 检查 UART 是否有待处理的中断条件
    /// 
    /// 用于 Bus 层在每次 UART 寄存器访问后更新 PLIC IRQ 10 状态。
    /// 模拟电平触发：当任何已使能的中断条件活跃时，UART 中断线为高。
    /// 
    /// # 返回
    /// - true: UART 有活跃中断，PLIC 应设置 IRQ 10 pending
    /// - false: 无活跃中断，PLIC 应清除 IRQ 10 pending
    pub fn has_pending_interrupt(&self) -> bool {
        let buffer = self.input_buffer.lock().unwrap();
        let has_input = !buffer.is_empty();
        drop(buffer);
        
        // 接收数据就绪中断 (ERBFI enabled AND data ready)
        if (self.ier & 0x01) != 0 && has_input {
            return true;
        }
        // 发送器保持寄存器空中断 (ETBEI enabled AND thr_ipending)
        if (self.ier & 0x02) != 0 && self.thr_ipending {
            return true;
        }
        false
    }
}
