// plic.rs - Platform-Level Interrupt Controller (PLIC)
// 完整实现：支持优先级、使能位和中断查询

/// PLIC 寄存器地址范围
/// Base: 0x0c00_0000
/// - Priority Registers: 0x000000 - 0x000FFF (1-1023 中断源，每个 4 字节)
/// - Pending Bits: 0x001000 - 0x00107F (每个 bit 表示一个中断)
/// - Enable Registers: 0x002000 - 0x01FFFF (每个 Context 有 32 个 32-bit 寄存器)
/// - Context Registers: 0x200000 - 0x3FFFFFF (每个 Context 0x1000 字节)
///   - Context 0 (M-Mode Hart 0): 0x200000
///   - Context 1 (S-Mode Hart 0): 0x201000

/// 最大支持的中断源数量
const MAX_IRQS: usize = 1024;
/// 最大支持的 Context 数量（M-Mode + S-Mode 各一个）
const MAX_CONTEXTS: usize = 2;

pub struct Plic {
    /// 中断优先级（IRQ 0 保留，IRQ 1-1023 可用）
    /// priority[irq] 表示 IRQ 的优先级 (0-7)，0 表示禁用
    priority: [u32; MAX_IRQS],
    
    /// Priority Threshold 寄存器（每个 Context 一个）
    /// Context 0 (M-Mode): offset 0x200000
    /// Context 1 (S-Mode): offset 0x201000
    /// 只有优先级 > threshold 的中断才会被传递
    priority_threshold: [u32; MAX_CONTEXTS],
    
    /// Claim/Complete 寄存器（每个 Context 一个）
    /// 读取：Claim 中断（返回中断 ID）
    /// 写入：Complete 中断（写入中断 ID）
    claim_complete: [u32; MAX_CONTEXTS],
    
    /// Pending 位（中断待处理标志）
    /// 与硬件一致的位映射：IRQ N = bit (N % 32) of pending[N / 32]
    /// pending[0] bit 0 = IRQ 0 (保留), bit 1 = IRQ 1, ..., bit 31 = IRQ 31
    /// pending[1] bit 0 = IRQ 32, ...
    pending: [u32; 32],
    
    /// Enable 位（每个 Context 一组，每组 32 个 32-bit 寄存器）
    /// 与硬件一致的位映射：IRQ N = bit (N % 32) of enable[context][N / 32]
    enable: [[u32; 32]; MAX_CONTEXTS],
}

impl Plic {
    pub fn new() -> Self {
        Self {
            priority: [0; MAX_IRQS],
            priority_threshold: [0; MAX_CONTEXTS],
            claim_complete: [0; MAX_CONTEXTS],
            pending: [0; 32],
            enable: [[0; 32]; MAX_CONTEXTS],
        }
    }
    
    /// 检查指定 Context 是否有待处理的中断
    /// 
    /// # 参数
    /// - `context_id`: Context ID (0 = M-Mode, 1 = S-Mode)
    /// 
    /// # 返回
    /// 如果有优先级 > threshold 且已使能的待处理中断，返回 true
    pub fn has_pending_interrupt(&self, context_id: usize) -> bool {
        if context_id >= MAX_CONTEXTS {
            return false;
        }
        
        let threshold = self.priority_threshold[context_id];
        
        // 遍历所有可能的 IRQ (1-1023)
        // 位映射与硬件一致：IRQ N = bit (N % 32) of register (N / 32)
        for idx in 0..32 {
            let pending_bits = self.pending[idx];
            let enable_bits = self.enable[context_id][idx];
            
            // 检查同时挂起且使能的中断
            let active_bits = pending_bits & enable_bits;
            if active_bits == 0 {
                continue;
            }
            
            // 遍历每个活跃的中断位
            for bit in 0..32 {
                if (active_bits & (1 << bit)) != 0 {
                    let irq = (idx * 32 + bit) as usize;  // 直接映射，与硬件一致
                    if irq == 0 { continue; }  // IRQ 0 保留，跳过
                    if irq < MAX_IRQS {
                        let prio = self.priority[irq];
                        // 只有优先级 > threshold 的中断才会被传递
                        if prio > threshold {
                            return true;
                        }
                    }
                }
            }
        }
        
        false
    }
    
    /// 获取待 Claim 的最高优先级中断 ID
    /// 
    /// # 参数
    /// - `context_id`: Context ID (0 = M-Mode, 1 = S-Mode)
    /// 
    /// # 返回
    /// 最高优先级的待处理中断 ID，如果没有则返回 0
    pub fn claim_interrupt(&mut self, context_id: usize) -> u32 {
        if context_id >= MAX_CONTEXTS {
            return 0;
        }
        
        let threshold = self.priority_threshold[context_id];
        let mut best_irq: u32 = 0;
        let mut best_priority: u32 = 0;
        
        // 遍历所有可能的 IRQ (1-1023)，找到最高优先级的中断
        // 位映射与硬件一致：IRQ N = bit (N % 32) of register (N / 32)
        for idx in 0..32 {
            let pending_bits = self.pending[idx];
            let enable_bits = self.enable[context_id][idx];
            let active_bits = pending_bits & enable_bits;
            
            if active_bits == 0 {
                continue;
            }
            
            for bit in 0..32 {
                if (active_bits & (1 << bit)) != 0 {
                    let irq = (idx * 32 + bit) as u32;  // 直接映射，与硬件一致
                    if irq == 0 { continue; }  // IRQ 0 保留，跳过
                    if (irq as usize) < MAX_IRQS {
                        let prio = self.priority[irq as usize];
                        if prio > threshold && prio > best_priority {
                            best_irq = irq;
                            best_priority = prio;
                        }
                    }
                }
            }
        }
        
        // 如果找到了中断，清除其 pending 位
        if best_irq > 0 {
            let idx = (best_irq / 32) as usize;
            let bit = best_irq % 32;
            self.pending[idx] &= !(1 << bit);
        }
        
        best_irq
    }
    
    /// 完成中断处理（Complete 操作）
    /// 对于电平触发中断源，complete 后如果源仍然活跃，
    /// 需要外部调用者（Bus）重新检查并 set_pending
    pub fn complete_interrupt(&mut self, _context_id: usize, _irq: u32) {
        // Complete 操作本身不需要额外工作，
        // 因为 update_uart_irq_public() 每条指令都会同步 UART 状态到 PLIC。
        // 但如果是电平触发中断，pending 位可能需要立即重新设置。
        // 这在 Bus 层的 update_uart_irq() 中处理。
    }
    
    /// 设置中断待处理位
    /// 
    /// # 参数
    /// - `irq`: 中断 ID (1-1023)
    /// 
    /// 注意：使用与硬件一致的位映射 —— IRQ N 对应 bit (N % 32) of register (N / 32)
    pub fn set_pending(&mut self, irq: u32) {
        if irq > 0 && irq <= 1023 {
            let idx = (irq / 32) as usize;
            let bit = irq % 32;
            self.pending[idx] |= 1 << bit;
        }
    }
    
    /// 清除中断待处理位
    /// 
    /// # 参数
    /// - `irq`: 中断 ID (1-1023)
    pub fn clear_pending(&mut self, irq: u32) {
        if irq > 0 && irq <= 1023 {
            let idx = (irq / 32) as usize;
            let bit = irq % 32;
            self.pending[idx] &= !(1 << bit);
        }
    }
    
    /// 读取 32 位寄存器
    pub fn read32(&mut self, offset: u64) -> Result<u32, &'static str> {
        match offset {
            // Priority Registers (0x000000 - 0x000FFF)
            // 每个 IRQ 一个 32-bit 优先级寄存器
            0x000000..=0x000FFF => {
                let irq = (offset / 4) as usize;
                if irq < MAX_IRQS {
                    Ok(self.priority[irq])
                } else {
                    Ok(0)
                }
            }
            
            // Pending Bits (0x001000 - 0x00107F)
            // 返回待处理的中断位
            0x001000..=0x00107F => {
                let idx = ((offset - 0x001000) / 4) as usize;
                if idx < self.pending.len() {
                    Ok(self.pending[idx])
                } else {
                    Ok(0)
                }
            }
            
            // Enable Registers (0x002000 - 0x01FFFF)
            // Context 0 (M-Mode): 0x002000 - 0x00207F
            // Context 1 (S-Mode): 0x002080 - 0x0020FF
            0x002000..=0x01FFFF => {
                let rel_offset = offset - 0x002000;
                let context = (rel_offset / 0x80) as usize;
                let idx = ((rel_offset % 0x80) / 4) as usize;
                
                if context < MAX_CONTEXTS && idx < 32 {
                    Ok(self.enable[context][idx])
                } else {
                    Ok(0)
                }
            }
            
            // Context 0 (M-Mode) Priority Threshold (0x200000)
            0x200000 => Ok(self.priority_threshold[0]),
            
            // Context 0 (M-Mode) Claim/Complete (0x200004)
            0x200004 => Ok(self.claim_interrupt(0)),
            
            // Context 1 (S-Mode) Priority Threshold (0x201000)
            0x201000 => Ok(self.priority_threshold[1]),
            
            // Context 1 (S-Mode) Claim/Complete (0x201004)
            0x201004 => Ok(self.claim_interrupt(1)),
            
            // 其他 Context 寄存器范围（静默处理）
            0x200000..=0x3FFFFFF => Ok(0),
            
            _ => Ok(0)
        }
    }
    
    /// 写入 32 位寄存器
    pub fn write32(&mut self, offset: u64, value: u32) -> Result<(), &'static str> {
        match offset {
            // Priority Registers (0x000000 - 0x000FFF)
            // 设置每个 IRQ 的优先级
            0x000000..=0x000FFF => {
                let irq = (offset / 4) as usize;
                if irq < MAX_IRQS {
                    // 优先级通常限制为 0-7 (3 位)，但完整实现可支持更多
                    self.priority[irq] = value & 0x7;
                }
                Ok(())
            }
            
            // Pending Bits (0x001000 - 0x00107F)
            // Pending 位通常由硬件设置，软件可以写 1 清除（W1C）
            // 在简化实现中，我们允许软件写入
            0x001000..=0x00107F => {
                let idx = ((offset - 0x001000) / 4) as usize;
                if idx < self.pending.len() {
                    // W1C (Write-1-to-Clear) 语义
                    self.pending[idx] &= !value;
                }
                Ok(())
            }
            
            // Enable Registers (0x002000 - 0x01FFFF)
            // Context 0 (M-Mode): 0x002000 - 0x00207F
            // Context 1 (S-Mode): 0x002080 - 0x0020FF
            0x002000..=0x01FFFF => {
                let rel_offset = offset - 0x002000;
                let context = (rel_offset / 0x80) as usize;
                let idx = ((rel_offset % 0x80) / 4) as usize;
                
                if context < MAX_CONTEXTS && idx < 32 {
                    self.enable[context][idx] = value;
                }
                Ok(())
            }
            
            // Context 0 (M-Mode) Priority Threshold (0x200000)
            0x200000 => {
                self.priority_threshold[0] = value & 0x7;
                Ok(())
            }
            
            // Context 0 (M-Mode) Claim/Complete (0x200004)
            0x200004 => {
                self.complete_interrupt(0, value);
                Ok(())
            }
            
            // Context 1 (S-Mode) Priority Threshold (0x201000)
            0x201000 => {
                self.priority_threshold[1] = value & 0x7;
                Ok(())
            }
            
            // Context 1 (S-Mode) Claim/Complete (0x201004)
            0x201004 => {
                self.complete_interrupt(1, value);
                Ok(())
            }
            
            // 其他 Context 寄存器范围（静默处理）
            0x200000..=0x3FFFFFF => Ok(()),
            
            _ => Ok(())
        }
    }

}

impl Default for Plic {
    fn default() -> Self {
        Self::new()
    }
}
