// cpu.rs - RISC-V 64-bit CPU 核心
// 严格对齐 QEMU TCI 行为，MD5 必须一致！

// 调试开关（如需重新打开，可在本文件内按需添加）

use crate::bus::Bus;
use crate::csr::{Csr, MCAUSE, MEPC, MIE, MIDELEG, MEDELEG, MIP, MSTATUS, MTVAL, MTVEC, SATP, SEPC, SSTATUS, STVEC, SCAUSE, STVAL, STIMECMP};
use crate::mmu::{AccessType, Mmu};
use crate::trap::{Exception, Interrupt, Trap};
use byteorder::{LittleEndian, WriteBytesExt};

/// CPU 特权模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    User = 0,
    Supervisor = 1,
    Hypervisor = 2,
    Machine = 3,
}

pub struct Cpu {
    /// 程序计数器
    pub pc: u64,
    /// 通用寄存器 x0-x31 (x0 恒为 0)
    pub regs: [u64; 32],
    /// 浮点寄存器 f0-f31 (支持 D 扩展，64位)
    pub f_regs: [u64; 32],
    /// CSR 寄存器
    pub csr: Csr,
    /// 当前特权模式
    pub mode: Mode,
    /// 总线
    pub bus: Bus,
    /// LR/SC Reservation Set (用于原子操作)
    /// Some(addr) 表示有有效的保留地址，None 表示无保留
    pub reservation: Option<u64>,
    /// 指令计数
    pub inst_count: u64,
}

impl Cpu {
    pub fn new(bus: Bus) -> Self {
        Self {
            pc: 0,
            regs: [0; 32],
            f_regs: [0; 32],
            csr: Csr::new(),
            mode: Mode::Machine,
            bus,
            reservation: None,  // 初始无保留
            inst_count: 0,
        }
    }
    
    /// 清除 Reservation Set
    /// 根据 RISC-V 规范，以下情况需要清除：
    /// 1. SC 指令执行（无论成功与否）
    /// 2. 任何普通 Store 指令（SB, SH, SW, SD 及压缩版本 C.SW, C.SD, C.SWSP, C.SDSP）
    /// 3. AMO 指令（AMOSWAP, AMOADD, AMOXOR, AMOAND, AMOOR, AMOMIN, AMOMAX 等）
    /// 4. SRET/MRET 指令
    /// 5. 异常或中断发生
    #[inline]
    pub fn clear_reservation(&mut self) {
        self.reservation = None;
    }
    
    /// 设置 Reservation Set（LR 指令调用）
    #[inline]
    pub fn set_reservation(&mut self, addr: u64) {
        // 对齐到双字边界（8字节对齐）
        // 这是简化的实现，真实硬件可能使用缓存行大小
        self.reservation = Some(addr & !0x7);
    }
    
    /// 检查地址是否匹配 Reservation Set（SC 指令调用）
    #[inline]
    pub fn check_reservation(&self, addr: u64) -> bool {
        match self.reservation {
            Some(reserved_addr) => (addr & !0x7) == reserved_addr,
            None => false,
        }
    }

    /// ============ 跨页安全内存访问 ============
    /// 当 load/store 跨越虚拟页边界时，需要对每个页面分别做 MMU 翻译。
    /// 这是因为相邻虚拟页可能映射到不相邻的物理页。
    
    /// 从虚拟地址读取 size 字节（1/2/4/8），自动处理跨页情况
    fn load_va(&mut self, va: u64, size: u64) -> Result<u64, (Trap, u64)> {
        let page_offset = va & 0xFFF;
        let satp = self.csr.read(SATP);
        let mstatus = self.csr.read(MSTATUS);
        
        if page_offset + size <= 0x1000 {
            // 快速路径：不跨页，单次翻译
            let pa = Mmu::translate(va, AccessType::Load, self.mode, satp, mstatus, &mut self.bus)
                .map_err(|trap| (trap, va))?;
            match size {
                1 => self.bus.read8(pa).map(|v| v as u64),
                2 => self.bus.read16(pa).map(|v| v as u64),
                4 => self.bus.read32(pa).map(|v| v as u64),
                8 => self.bus.read64(pa),
                _ => unreachable!(),
            }
            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))
        } else {
            // 慢速路径：跨页，逐字节读取，每字节独立翻译
            let mut result: u64 = 0;
            for i in 0..size {
                let byte_va = va.wrapping_add(i);
                let pa = Mmu::translate(byte_va, AccessType::Load, self.mode, satp, mstatus, &mut self.bus)
                    .map_err(|trap| (trap, va))?;
                let byte = self.bus.read8(pa)
                    .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                result |= (byte as u64) << (i * 8);
            }
            Ok(result)
        }
    }
    
    /// 向虚拟地址写入 size 字节（1/2/4/8），自动处理跨页情况
    fn store_va(&mut self, va: u64, value: u64, size: u64) -> Result<(), (Trap, u64)> {
        let page_offset = va & 0xFFF;
        let satp = self.csr.read(SATP);
        let mstatus = self.csr.read(MSTATUS);
        
        if page_offset + size <= 0x1000 {
            // 快速路径：不跨页，单次翻译
            let pa = Mmu::translate(va, AccessType::Store, self.mode, satp, mstatus, &mut self.bus)
                .map_err(|trap| (trap, va))?;
            match size {
                1 => self.bus.write8(pa, value as u8),
                2 => self.bus.write16(pa, value as u16),
                4 => self.bus.write32(pa, value as u32),
                8 => self.bus.write64(pa, value),
                _ => unreachable!(),
            }
            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))
        } else {
            // 慢速路径：跨页，逐字节写入，每字节独立翻译
            for i in 0..size {
                let byte_va = va.wrapping_add(i);
                let byte = (value >> (i * 8)) as u8;
                let pa = Mmu::translate(byte_va, AccessType::Store, self.mode, satp, mstatus, &mut self.bus)
                    .map_err(|trap| (trap, va))?;
                self.bus.write8(pa, byte)
                    .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
            }
            Ok(())
        }
    }

    /// 读取通用寄存器 (x0 恒为 0)
    pub fn read_reg(&self, index: usize) -> u64 {
        if index == 0 {
            0
        } else {
            self.regs[index]
        }
    }

    /// 写入通用寄存器 (x0 恒为 0，写入无效)
    pub fn write_reg(&mut self, index: usize, value: u64) {
        if index != 0 {
            self.regs[index] = value;
        }
    }

    /// ============ CPU 执行主循环 ============
    /// 严格遵循顺序：Check Interrupt -> Fetch -> Decode -> Execute -> Timer Update
    pub fn step(&mut self) -> Result<(), Trap> {
        // 0. 更新指令计数
        self.inst_count += 1;

        // 1. 检查中断
        if let Err(trap) = self.check_interrupt() {
            self.handle_trap(trap, 0);
            return Ok(());
        }

        // 2. 取指令（支持16位和32位）
        let inst = match self.fetch() {
            Ok(i) => i,
            Err((trap, tval)) => {
                self.handle_trap(trap, tval);
                return Ok(());
            }
        };

        // 3. 译码 & 执行
        if let Err((trap, tval)) = self.execute(inst) {
            // ⚠️ 关键修复：直接使用 execute 返回的 tval
            // 对于 IllegalInstruction，tval 是指令本身
            // 对于访存异常，tval 是触发异常的虚拟地址
            self.handle_trap(trap, tval);
            return Ok(());
        }

        Ok(())
    }

    /// 处理异常/中断 (严格遵循 RISC-V 特权级规范)
    pub fn handle_trap(&mut self, trap: Trap, tval: u64) -> bool {
        // 调试输出：只打印真正异常的情况，过滤掉正常操作
        match &trap {
            Trap::Exception(exc) => {
                let should_print = match exc {
                    // 正常操作，永不打印
                    Exception::EnvironmentCallFromSMode |
                    Exception::EnvironmentCallFromUMode |
                    Exception::Breakpoint => false,
                    
                    // 非法指令：过滤掉已知的 CSR 探测
                    Exception::IllegalInstruction => {
                        let csr_addr = (tval >> 20) & 0xFFF;
                        !(csr_addr >= 0xB00 && csr_addr <= 0xBFF) &&  // HPM counters
                        !(csr_addr >= 0x300 && csr_addr <= 0x3FF) &&  // Machine CSRs 探测
                        !(csr_addr >= 0x100 && csr_addr <= 0x1FF) &&  // Supervisor CSRs 探测
                        !(csr_addr >= 0xD00 && csr_addr <= 0xDFF) &&  // Debug CSRs
                        !(csr_addr >= 0xF00 && csr_addr <= 0xFFF)     // Vendor CSRs
                    }
                    
                    //  PageFault = 正常的 demand paging，全部不打印
                    // User 模式：mmap、堆分配、CoW 等标准延迟分页
                    // Supervisor 模式：内核在 syscall 中访问用户页面（__clear_user、copy_to_user 等）
                    // 两者都是 Linux 正常行为，会产生海量输出淹没终端。
                    Exception::InstructionPageFault |
                    Exception::LoadPageFault |
                    Exception::StoreAMOPageFault => false,
                    
                    // 其他异常（AccessFault 等）始终打印
                    _ => true,
                };
                
                if should_print {
                    eprintln!("\x1b[31m[CPU-EXCEPTION] {:?} at PC=0x{:016X} tval=0x{:016X} Mode={:?} Inst#{}\x1b[0m",
                        exc, self.pc, tval, self.mode, self.inst_count);
                }
            }
            Trap::Interrupt(_int) => {
                // 中断是正常操作，不输出避免刷屏
                // 如需调试可取消注释下行：
                // println!("[CPU-INTERRUPT] {:?} at PC=0x{:016X} Mode={:?}", _int, self.pc, self.mode);
            }
        }
        
        // 异常/中断发生时清除 Reservation Set
        // 这是 RISC-V 规范要求的行为
        self.clear_reservation();
       
        let mideleg = self.csr.read(MIDELEG);
        let medeleg = self.csr.read(MEDELEG);
        
        // ⚠️ 关键修复：中断和异常的委托判断
        // 使用 trap.code() 提取标准的 RISC-V 异常代码，并去除最高位中断标志
        let code = trap.code();
        let exception_code = code & !(1u64 << 63);  // 去除中断标志位
        
        let should_delegate = match trap {
            // 对于中断，检查 mideleg 中对应的位
            Trap::Interrupt(_) => {
                (mideleg >> exception_code) & 1 != 0
            }
            // 对于异常，检查 medeleg 中对应的位
            Trap::Exception(_) => {
                (medeleg >> exception_code) & 1 != 0
            }
        };

        // 只有在非 M-Mode 且异常/中断被委托时，才进入 S-Mode trap 处理
        if should_delegate && self.mode != Mode::Machine {
            self.take_s_trap(trap, tval);
        } else {
            self.take_trap(trap, tval);
        }
        true
    }

    /// 执行 S-mode Trap 入口
    fn take_s_trap(&mut self, trap: Trap, mtval: u64) {
        let pc = self.pc;
        let stvec = self.csr.read(STVEC);
        let scause = trap.code();

        self.csr.write(SEPC, pc);
        self.csr.write(SCAUSE, scause);
        self.csr.write(STVAL, mtval);

        let mut sstatus = self.csr.read(SSTATUS);
        let sie = (sstatus >> 1) & 0x1;
        sstatus = (sstatus & !(1 << 5)) | (sie << 5); // SPIE <- SIE
        sstatus &= !(1 << 1); // SIE <- 0
        sstatus = (sstatus & !(1 << 8)) | (((self.mode as u64) & 1) << 8); // SPP
        self.csr.write(SSTATUS, sstatus);

        self.mode = Mode::Supervisor;
        let base = stvec & !0x3;
        if trap.is_interrupt() && (stvec & 0x1) != 0 {
            self.pc = base.wrapping_add(4 * (scause & 0x3F));
        } else {
            self.pc = base;
        }
    }

    /// 执行 M-mode Trap 入口
    fn take_trap(&mut self, trap: Trap, mtval: u64) {
        let pc = self.pc;
        let mtvec = self.csr.read(MTVEC);
        let mcause = trap.code();

        self.csr.write(MEPC, pc);
        self.csr.write(MCAUSE, mcause);
        self.csr.write(MTVAL, mtval);

        let mut mstatus = self.csr.read(MSTATUS);
        let mie = (mstatus >> 3) & 0x1;
        mstatus = (mstatus & !(1 << 7)) | (mie << 7); // MPIE <- MIE
        mstatus &= !(1 << 3); // MIE <- 0
        mstatus = (mstatus & !(0x3 << 11)) | ((self.mode as u64) << 11); // MPP
        self.csr.write(MSTATUS, mstatus);

        self.mode = Mode::Machine;
        let base = mtvec & !0x3;
        if trap.is_interrupt() && (mtvec & 0x1) != 0 {
            self.pc = base.wrapping_add(4 * (mcause & 0x3F));
        } else {
            self.pc = base;
        }
    }

    /// 取指令 (支持 16-bit 压缩指令和 32-bit 标准指令)
    /// 返回 Result<u32, (Trap, u64)>，其中 u64 是触发异常的虚拟地址 (tval)
    fn fetch(&mut self) -> Result<u32, (Trap, u64)> {
        // 检查 PC 是否 2 字节对齐（压缩指令要求）
        if self.pc % 2 != 0 {
            return Err((Trap::Exception(Exception::InstructionAddressMisaligned), self.pc));
        }

        // 通过 MMU 转换虚拟地址到物理地址
        let satp = self.csr.read(SATP);
        let mstatus = self.csr.read(MSTATUS);
        let pa = Mmu::translate(self.pc, AccessType::Instruction, self.mode, satp, mstatus, &mut self.bus)
            .map_err(|trap| (trap, self.pc))?;  // ⚠️ 关键修复：传递 PC 作为 tval

        // 先读取 16 位
        let lower = match self.bus.read16(pa) {
            Ok(v) => v as u32,
            Err(_) => return Err((Trap::Exception(Exception::InstructionAccessFault), self.pc)),
        };

        // 检查最低 2 位判断是否为压缩指令
        if (lower & 0x3) != 0x3 {
            // 压缩指令 (16-bit)
            Ok(lower)
        } else {
            // 标准指令 (32-bit)：读取上半部分 16 位
            // 当 32 位指令跨越 4KB 页面边界时，
            // 上半部分可能映射到不同的物理页面，必须单独做 MMU 翻译！
            // 例如：PC=0x92ffe 的低 16 位在页 0x92xxx，高 16 位在页 0x93xxx，
            // 这两个虚拟页可能映射到完全不同的物理页。
            let upper_pa = if (self.pc & 0xFFF) >= 0xFFE {
                // 指令跨页！上半部分在下一个虚拟页，需要单独翻译
                let upper_va = self.pc.wrapping_add(2);
                Mmu::translate(upper_va, AccessType::Instruction, self.mode, satp, mstatus, &mut self.bus)
                    .map_err(|trap| (trap, self.pc))?
            } else {
                // 同一页内，物理地址连续
                pa + 2
            };
            let upper = match self.bus.read16(upper_pa) {
                Ok(v) => v as u32,
                Err(_) => return Err((Trap::Exception(Exception::InstructionAccessFault), self.pc)),
            };
            Ok(lower | (upper << 16))
        }
    }

    /// 译码并执行指令
    /// 返回 Result<(), (Trap, u64)>，其中 u64 是触发异常的虚拟地址 (tval)
    fn execute(&mut self, inst: u32) -> Result<(), (Trap, u64)> {
        // ============ 判断指令长度 ============
        let inst_len = if (inst & 0x3) != 0x3 { 2 } else { 4 };
        
        // ============ 处理压缩指令 ============
        if inst_len == 2 {
            let result = self.execute_compressed(inst as u16);
            return result;
        }

        // ============ 标准指令格式解析 ============
        let opcode = inst & 0x7F;
        let rd = ((inst >> 7) & 0x1F) as usize;
        let funct3 = (inst >> 12) & 0x07;
        let rs1 = ((inst >> 15) & 0x1F) as usize;
        let rs2 = ((inst >> 20) & 0x1F) as usize;
        let funct7 = (inst >> 25) & 0x7F;

        // I-type 立即数
        let imm_i = ((inst as i32) >> 20) as i64 as u64;
        // S-type 立即数
        let imm_s = (((inst & 0xFE000000) as i32 >> 20) as i64 | ((inst >> 7) & 0x1F) as i64) as u64;
        // B-type 立即数
        let imm_b = (((inst & 0x80000000) as i32 >> 19) as i64
            | (((inst & 0x80) << 4) as i64)
            | (((inst >> 20) & 0x7E0) as i64)
            | (((inst >> 7) & 0x1E) as i64)) as u64;
        // U-type 立即数（RV64 需要对 32-bit 结果做符号扩展）
        let imm_u = (inst & 0xFFFFF000) as i32 as i64 as u64;
        // J-type 立即数
        let imm_j = (((inst & 0x80000000) as i32 >> 11) as i64
            | ((inst & 0xFF000) as i64)
            | (((inst >> 9) & 0x800) as i64)
            | (((inst >> 20) & 0x7FE) as i64)) as u64;

        // 保存当前 PC，用于分支/跳转指令
        let next_pc = self.pc.wrapping_add(4);

        match opcode {
            // ============ LUI (Load Upper Immediate) ============
            0x37 => {
                self.write_reg(rd, imm_u);
                self.pc = next_pc;
            }

            // ============ AUIPC (Add Upper Immediate to PC) ============
            0x17 => {
                self.write_reg(rd, self.pc.wrapping_add(imm_u));
                self.pc = next_pc;
            }

            // ============ JAL (Jump and Link) ============
            0x6F => {
                self.write_reg(rd, next_pc);
                self.pc = self.pc.wrapping_add(imm_j);
            }

            // ============ JALR (Jump and Link Register) ============
            0x67 => {
                let target = self.read_reg(rs1).wrapping_add(imm_i) & !1;
                self.write_reg(rd, next_pc);
                self.pc = target;
            }

            // ============ 分支指令 (Branch) ============
            0x63 => {
                let rs1_val = self.read_reg(rs1);
                let rs2_val = self.read_reg(rs2);
                let branch_taken = match funct3 {
                    0x0 => rs1_val == rs2_val,                   // BEQ
                    0x1 => rs1_val != rs2_val,                   // BNE
                    0x4 => (rs1_val as i64) < (rs2_val as i64),  // BLT
                    0x5 => (rs1_val as i64) >= (rs2_val as i64), // BGE
                    0x6 => rs1_val < rs2_val,                    // BLTU
                    0x7 => rs1_val >= rs2_val,                   // BGEU
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                };

                if branch_taken {
                    self.pc = self.pc.wrapping_add(imm_b);
                } else {
                    self.pc = next_pc;
                }
            }

            // ============ 加载指令 (Load) ============
            // ⚠️ 使用 load_va 处理跨页非对齐访问
            0x03 => {
                let va = self.read_reg(rs1).wrapping_add(imm_i);
                
                let value = match funct3 {
                    0x0 => { let v = self.load_va(va, 1)?; (v as u8) as i8 as i64 as u64 },   // LB
                    0x1 => { let v = self.load_va(va, 2)?; (v as u16) as i16 as i64 as u64 }, // LH
                    0x2 => { let v = self.load_va(va, 4)?; (v as u32) as i32 as i64 as u64 }, // LW
                    0x3 => self.load_va(va, 8)?,                                               // LD
                    0x4 => self.load_va(va, 1)?,                                               // LBU
                    0x5 => self.load_va(va, 2)?,                                               // LHU
                    0x6 => self.load_va(va, 4)?,                                               // LWU
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                };

                self.write_reg(rd, value);
                self.pc = next_pc;
            }

            // ============ 浮点加载指令 (LOAD-FP) ============
            // ⚠️ 使用 load_va 处理跨页非对齐访问
            0x07 => {
                let va = self.read_reg(rs1).wrapping_add(imm_i);
                
                match funct3 {
                    0x2 => {
                        // FLW - 加载单精度浮点 (32位)
                        let value = self.load_va(va, 4)?;
                        // NaN-boxing: 高32位全1，低32位是加载的值
                        self.f_regs[rd] = 0xFFFF_FFFF_0000_0000 | (value & 0xFFFF_FFFF);
                    }
                    0x3 => {
                        // FLD - 加载双精度浮点 (64位)
                        let value = self.load_va(va, 8)?;
                        self.f_regs[rd] = value;
                    }
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                }
                
                // 设置 mstatus.FS = Dirty (3)
                let mut mstatus = self.csr.read(MSTATUS);
                mstatus = (mstatus & !(0x3 << 13)) | (0x3 << 13);
                self.csr.write(MSTATUS, mstatus);
                
                self.pc = next_pc;
            }

            // ============ 存储指令 (Store) ============
            // 使用 store_va 处理跨页非对齐访问
            0x23 => {
                let va = self.read_reg(rs1).wrapping_add(imm_s);
                let value = self.read_reg(rs2);
                
                match funct3 {
                    0x0 => self.store_va(va, value, 1)?,  // SB
                    0x1 => self.store_va(va, value, 2)?,  // SH
                    0x2 => self.store_va(va, value, 4)?,  // SW
                    0x3 => self.store_va(va, value, 8)?,  // SD
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                }

                // Store 指令清除 Reservation Set
                self.clear_reservation();
                self.pc = next_pc;
            }

            // ============ 浮点存储指令 (STORE-FP) ============
            // 使用 store_va 处理跨页非对齐访问
            0x27 => {
                let va = self.read_reg(rs1).wrapping_add(imm_s);
                
                match funct3 {
                    0x2 => {
                        // FSW - 存储单精度浮点 (32位)
                        let value = self.f_regs[rs2] & 0xFFFF_FFFF;
                        self.store_va(va, value, 4)?;
                    }
                    0x3 => {
                        // FSD - 存储双精度浮点 (64位)
                        let value = self.f_regs[rs2];
                        self.store_va(va, value, 8)?;
                    }
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                }
                
                // 浮点存储也清除 reservation
                self.clear_reservation();
                self.pc = next_pc;
            }

            // ============ 立即数算术指令 (I-type) ============
            0x13 => {
                let rs1_val = self.read_reg(rs1);
                let shamt = (imm_i & 0x3F) as u32; // 6-bit for RV64
                let imm12 = (inst >> 20) & 0xFFF; // 完整的 12 位立即数用于 Zbb 扩展检测
                let result = match funct3 {
                    0x0 => rs1_val.wrapping_add(imm_i), // ADDI
                    0x1 => {
                        // 检查是否为 Zbb/Zbs 扩展指令
                        let funct7_like = (imm12 >> 5) & 0x7F; // 高7位用于区分扩展指令
                        match imm12 {
                            0x600 => {
                                // CLZ - Count Leading Zeros (Zbb)
                                rs1_val.leading_zeros() as u64
                            }
                            0x601 => {
                                // CTZ - Count Trailing Zeros (Zbb)
                                rs1_val.trailing_zeros() as u64
                            }
                            0x602 => {
                                // CPOP - Count Population (Zbb)
                                rs1_val.count_ones() as u64
                            }
                            0x604 => {
                                // SEXT.B - Sign-extend Byte (Zbb)
                                (rs1_val as i8) as i64 as u64
                            }
                            0x605 => {
                                // SEXT.H - Sign-extend Halfword (Zbb)
                                (rs1_val as i16) as i64 as u64
                            }
                            _ => {
                                // 检查 Zbs 扩展的立即数版本
                                match funct7_like {
                                    0x24 => {
                                        // BCLRI - 清除指定位
                                        rs1_val & !(1u64 << shamt)
                                    }
                                    0x34 => {
                                        // BINVI - 翻转指定位
                                        rs1_val ^ (1u64 << shamt)
                                    }
                                    0x14 => {
                                        // BSETI - 设置指定位
                                        rs1_val | (1u64 << shamt)
                                    }
                                    _ => {
                                        // 标准 SLLI
                                        rs1_val << shamt
                                    }
                                }
                            }
                        }
                    }
                    0x2 => {
                        if (rs1_val as i64) < (imm_i as i64) {
                            1
                        } else {
                            0
                        }
                    } // SLTI
                    0x3 => {
                        if rs1_val < imm_i {
                            1
                        } else {
                            0
                        }
                    } // SLTIU
                    0x4 => {
                        // 检查是否为 Zbb 扩展指令
                        if imm12 == 0x287 {
                            // ZEXT.H - Zero-extend Halfword (Zbb, 在 RV64 中)
                            // 注意：ZEXT.H 在 OP-IMM 中编码不同，这里可能需要调整
                            rs1_val & 0xFFFF
                        } else {
                            rs1_val ^ imm_i // XORI
                        }
                    }
                    0x5 => {
                        // 检查是否为 Zbb/Zbs 扩展指令
                        let funct7_like = (imm12 >> 5) & 0x7F; // 高7位用于区分扩展指令
                        match imm12 {
                            0x287 => {
                                // ORC.B - OR-Combine Bytes (Zbb)
                                let mut result = 0u64;
                                for i in 0..8 {
                                    let byte = (rs1_val >> (i * 8)) & 0xFF;
                                    if byte != 0 {
                                        result |= 0xFF << (i * 8);
                                    }
                                }
                                result
                            }
                            0x6B8 => {
                                // REV8 - Byte-reverse (Zbb)
                                rs1_val.swap_bytes()
                            }
                            _ => {
                                // 检查 Zbs/Zbb 扩展
                                match funct7_like {
                                    0x24 => {
                                        // BEXTI - 提取指定位
                                        (rs1_val >> shamt) & 1
                                    }
                                    0x30 => {
                                        // RORI - 循环右移立即数
                                        rs1_val.rotate_right(shamt as u32)
                                    }
                                    _ => {
                                        // RV64I: shamt 是 6 位 (bit 25:20)，只检查 bit 30 区分 SRLI/SRAI
                                        if (inst >> 30) & 1 == 0 {
                                            rs1_val >> shamt // SRLI
                                        } else {
                                            ((rs1_val as i64) >> shamt) as u64 // SRAI
                                        }
                                    }
                                }
                            }
                        }
                    }
                    0x6 => rs1_val | imm_i,  // ORI
                    0x7 => rs1_val & imm_i,  // ANDI
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                };
                self.write_reg(rd, result);
                self.pc = next_pc;
            }

            // ============ 寄存器算术指令 (R-type) ============
            0x33 => {
                let rs1_val = self.read_reg(rs1);
                let rs2_val = self.read_reg(rs2);
                let result = match (funct7, funct3) {
                    (0x00, 0x0) => rs1_val.wrapping_add(rs2_val), // ADD
                    (0x20, 0x0) => rs1_val.wrapping_sub(rs2_val), // SUB
                    (0x00, 0x1) => rs1_val << (rs2_val & 0x3F),   // SLL
                    (0x00, 0x2) => {
                        if (rs1_val as i64) < (rs2_val as i64) {
                            1
                        } else {
                            0
                        }
                    } // SLT
                    (0x00, 0x3) => {
                        if rs1_val < rs2_val {
                            1
                        } else {
                            0
                        }
                    } // SLTU
                    (0x00, 0x4) => rs1_val ^ rs2_val,                           // XOR
                    (0x00, 0x5) => rs1_val >> (rs2_val & 0x3F),                 // SRL
                    (0x20, 0x5) => ((rs1_val as i64) >> (rs2_val & 0x3F)) as u64, // SRA
                    (0x00, 0x6) => rs1_val | rs2_val,                           // OR
                    (0x00, 0x7) => rs1_val & rs2_val,                           // AND
                    // M 扩展 (RV64)
                    (0x01, 0x0) => (rs1_val as i64).wrapping_mul(rs2_val as i64) as u64, // MUL
                    (0x01, 0x1) => {
                        let result = (rs1_val as i64 as i128) * (rs2_val as i64 as i128);
                        (result >> 64) as u64
                    } // MULH
                    (0x01, 0x2) => {
                        let result = (rs1_val as i64 as i128) * (rs2_val as u64 as u128 as i128);
                        (result >> 64) as u64
                    } // MULHSU
                    (0x01, 0x3) => {
                        let result = (rs1_val as u128) * (rs2_val as u128);
                        (result >> 64) as u64
                    } // MULHU
                    (0x01, 0x4) => {
                        let a = rs1_val as i64;
                        let b = rs2_val as i64;
                        if b == 0 {
                            u64::MAX
                        } else if a == i64::MIN && b == -1 {
                            a as u64
                        } else {
                            (a / b) as u64
                        }
                    } // DIV
                    (0x01, 0x5) => {
                        if rs2_val == 0 {
                            u64::MAX
                        } else {
                            rs1_val / rs2_val
                        }
                    } // DIVU
                    (0x01, 0x6) => {
                        let a = rs1_val as i64;
                        let b = rs2_val as i64;
                        if b == 0 {
                            a as u64
                        } else if a == i64::MIN && b == -1 {
                            0
                        } else {
                            (a % b) as u64
                        }
                    } // REM
                    (0x01, 0x7) => {
                        if rs2_val == 0 {
                            rs1_val
                        } else {
                            rs1_val % rs2_val
                        }
                    } // REMU
                    
                    // ============ Zbb 扩展 (位操作基础) ============
                    (0x20, 0x4) => !rs1_val ^ rs2_val,  // XNOR: ~(rs1 ^ rs2) = ~rs1 ^ rs2
                    (0x20, 0x6) => rs1_val | !rs2_val,  // ORN: rs1 | ~rs2
                    (0x20, 0x7) => rs1_val & !rs2_val,  // ANDN: rs1 & ~rs2
                    (0x05, 0x4) => {
                        // MIN - 有符号最小值
                        std::cmp::min(rs1_val as i64, rs2_val as i64) as u64
                    }
                    (0x05, 0x5) => {
                        // MINU - 无符号最小值
                        std::cmp::min(rs1_val, rs2_val)
                    }
                    (0x05, 0x6) => {
                        // MAX - 有符号最大值
                        std::cmp::max(rs1_val as i64, rs2_val as i64) as u64
                    }
                    (0x05, 0x7) => {
                        // MAXU - 无符号最大值
                        std::cmp::max(rs1_val, rs2_val)
                    }
                    (0x30, 0x1) => {
                        // ROL - 64位循环左移
                        let shamt = (rs2_val & 0x3F) as u32;
                        rs1_val.rotate_left(shamt)
                    }
                    (0x30, 0x5) => {
                        // ROR - 64位循环右移
                        let shamt = (rs2_val & 0x3F) as u32;
                        rs1_val.rotate_right(shamt)
                    }
                    
                    // ============ Zba 扩展 (地址计算) ============
                    (0x10, 0x2) => {
                        // SH1ADD: (rs1 << 1) + rs2
                        (rs1_val << 1).wrapping_add(rs2_val)
                    }
                    (0x10, 0x4) => {
                        // SH2ADD: (rs1 << 2) + rs2
                        (rs1_val << 2).wrapping_add(rs2_val)
                    }
                    (0x10, 0x6) => {
                        // SH3ADD: (rs1 << 3) + rs2
                        (rs1_val << 3).wrapping_add(rs2_val)
                    }
                    
                    // ============ Zbs 扩展 (单比特操作) ============
                    (0x24, 0x1) => {
                        // BCLR - 清除指定位
                        let shamt = rs2_val & 0x3F;
                        rs1_val & !(1u64 << shamt)
                    }
                    (0x24, 0x5) => {
                        // BEXT - 提取指定位
                        let shamt = rs2_val & 0x3F;
                        (rs1_val >> shamt) & 1
                    }
                    (0x34, 0x1) => {
                        // BINV - 翻转指定位
                        let shamt = rs2_val & 0x3F;
                        rs1_val ^ (1u64 << shamt)
                    }
                    (0x14, 0x1) => {
                        // BSET - 设置指定位
                        let shamt = rs2_val & 0x3F;
                        rs1_val | (1u64 << shamt)
                    }
                    
                    // ============ Zbc 扩展 (无进位乘法) ============
                    (0x05, 0x1) => {
                        // CLMUL - 无进位乘法低位
                        let mut result = 0u64;
                        for i in 0..64 {
                            if (rs2_val >> i) & 1 == 1 {
                                result ^= rs1_val << i;
                            }
                        }
                        result
                    }
                    (0x05, 0x3) => {
                        // CLMULH - 无进位乘法高位
                        let mut result = 0u64;
                        for i in 1..64 {
                            if (rs2_val >> i) & 1 == 1 {
                                result ^= rs1_val >> (64 - i);
                            }
                        }
                        result
                    }
                    (0x05, 0x2) => {
                        // CLMULR - 无进位乘法反转
                        let mut result = 0u64;
                        for i in 0..64 {
                            if (rs2_val >> i) & 1 == 1 {
                                result ^= rs1_val >> (63 - i);
                            }
                        }
                        result
                    }
                    
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                };
                self.write_reg(rd, result);
                self.pc = next_pc;
            }

            // ============ 32-bit 立即数算术指令 (I-type W) ============
            0x1B => {
                let rs1_val = self.read_reg(rs1) as u32;
                let shamt = (imm_i & 0x1F) as u32; // 5-bit for RV64 W instructions
                let imm12 = (inst >> 20) & 0xFFF; // 完整的 12 位立即数用于 Zbb 扩展检测
                let result = match funct3 {
                    0x0 => rs1_val.wrapping_add(imm_i as u32),        // ADDIW
                    0x1 => {
                        // 检查是否为 Zbb/Zba 扩展指令
                        let funct7_like = (imm12 >> 5) & 0x7F;
                        match imm12 {
                            0x600 => {
                                // CLZW - Count Leading Zeros Word (Zbb)
                                rs1_val.leading_zeros()
                            }
                            0x601 => {
                                // CTZW - Count Trailing Zeros Word (Zbb)
                                rs1_val.trailing_zeros()
                            }
                            0x602 => {
                                // CPOPW - Count Population Word (Zbb)
                                rs1_val.count_ones()
                            }
                            _ => {
                                // 检查是否为 Zba 扩展 SLLI.UW
                                if funct7_like == 0x04 {
                                    // SLLI.UW - 零扩展后左移 (Zba)
                                    // rs1 的低 32 位零扩展到 64 位，然后左移 shamt 位
                                    // 结果是 64 位，不需要符号扩展
                                    let rs1_full = self.read_reg(rs1);
                                    let shamt6 = (imm_i & 0x3F) as u32; // SLLI.UW 使用 6 位 shamt
                                    return {
                                        let result = ((rs1_full as u32) as u64) << shamt6;
                                        self.write_reg(rd, result);
                                        self.pc = next_pc;
                                        Ok(())
                                    };
                                } else {
                                    // 标准 SLLIW
                                    rs1_val << shamt
                                }
                            }
                        }
                    }
                    0x5 => {
                        // 检查 bit 30 区分 SRLIW/SRAIW 和 Zbb 扩展
                        match imm12 {
                            0x604 => {
                                // SEXT.B - Sign-extend Byte (Zbb) - 但这不应该在 W 指令中
                                // 保留用于其他扩展
                                return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                            }
                            0x605 => {
                                // SEXT.H - Sign-extend Halfword (Zbb) - 但这不应该在 W 指令中
                                return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                            }
                            0x698 => {
                                // RORIW - Rotate Right Word Immediate (Zbb)
                                let shamt5 = shamt & 0x1F;
                                rs1_val.rotate_right(shamt5)
                            }
                            _ => {
                                if (inst >> 30) & 1 == 0 {
                                    rs1_val >> shamt // SRLIW
                                } else {
                                    ((rs1_val as i32) >> shamt) as u32 // SRAIW
                                }
                            }
                        }
                    }
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                } as i32 as i64 as u64;
                self.write_reg(rd, result);
                self.pc = next_pc;
            }

            // ============ 32-bit 寄存器算术指令 (R-type W) ============
            0x3B => {
                let rs1_val = self.read_reg(rs1) as u32;
                let rs2_val = self.read_reg(rs2) as u32;
                let result = match (funct7, funct3) {
                    (0x00, 0x0) => rs1_val.wrapping_add(rs2_val),       // ADDW
                    (0x20, 0x0) => rs1_val.wrapping_sub(rs2_val),       // SUBW
                    (0x00, 0x1) => rs1_val << (rs2_val & 0x1F),         // SLLW
                    (0x00, 0x5) => rs1_val >> (rs2_val & 0x1F),         // SRLW
                    (0x20, 0x5) => ((rs1_val as i32) >> (rs2_val & 0x1F)) as u32, // SRAW
                    // M 扩展 (RV64M - W 指令)
                    (0x01, 0x0) => {
                        let value = (rs1_val as i32 as i64)
                            .wrapping_mul(rs2_val as i32 as i64) as i32;
                        value as u32
                    } // MULW
                    (0x01, 0x4) => {
                        let a = rs1_val as i32;
                        let b = rs2_val as i32;
                        let value = if b == 0 {
                            -1
                        } else if a == i32::MIN && b == -1 {
                            a
                        } else {
                            a / b
                        };
                        value as u32
                    } // DIVW
                    (0x01, 0x5) => {
                        let a = rs1_val;
                        let b = rs2_val;
                        if b == 0 { u32::MAX } else { a / b }
                    } // DIVUW
                    (0x01, 0x6) => {
                        let a = rs1_val as i32;
                        let b = rs2_val as i32;
                        let value = if b == 0 {
                            a
                        } else if a == i32::MIN && b == -1 {
                            0
                        } else {
                            a % b
                        };
                        value as u32
                    } // REMW
                    (0x01, 0x7) => {
                        let a = rs1_val;
                        let b = rs2_val;
                        if b == 0 { a } else { a % b }
                    } // REMUW
                    
                    // ============ Zbb 扩展 (位操作基础 - W版本) ============
                    (0x30, 0x1) => {
                        // ROLW - 32位循环左移
                        let shamt = (rs2_val & 0x1F) as u32;
                        rs1_val.rotate_left(shamt)
                    }
                    (0x30, 0x5) => {
                        // RORW - 32位循环右移
                        let shamt = (rs2_val & 0x1F) as u32;
                        rs1_val.rotate_right(shamt)
                    }
                    
                    // ============ Zba 扩展 (地址计算 - UW版本) ============
                    (0x04, 0x0) => {
                        // ADD.UW: 零扩展 rs1[31:0] 然后加 rs2
                        // rs1_val 已经是 u32, 加上完整的 rs2
                        let rs2_full = self.read_reg(rs2);
                        return {
                            let result = (rs1_val as u64).wrapping_add(rs2_full);
                            self.write_reg(rd, result);
                            self.pc = next_pc;
                            Ok(())
                        };
                    }
                    (0x10, 0x2) => {
                        // SH1ADD.UW: ((rs1 as u32) << 1) + rs2
                        let rs2_full = self.read_reg(rs2);
                        return {
                            let result = ((rs1_val as u64) << 1).wrapping_add(rs2_full);
                            self.write_reg(rd, result);
                            self.pc = next_pc;
                            Ok(())
                        };
                    }
                    (0x10, 0x4) => {
                        // SH2ADD.UW: ((rs1 as u32) << 2) + rs2
                        let rs2_full = self.read_reg(rs2);
                        return {
                            let result = ((rs1_val as u64) << 2).wrapping_add(rs2_full);
                            self.write_reg(rd, result);
                            self.pc = next_pc;
                            Ok(())
                        };
                    }
                    (0x10, 0x6) => {
                        // SH3ADD.UW: ((rs1 as u32) << 3) + rs2
                        let rs2_full = self.read_reg(rs2);
                        return {
                            let result = ((rs1_val as u64) << 3).wrapping_add(rs2_full);
                            self.write_reg(rd, result);
                            self.pc = next_pc;
                            Ok(())
                        };
                    }
                    
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                } as i32 as i64 as u64;
                self.write_reg(rd, result);
                self.pc = next_pc;
            }

            // ============ MISC-MEM 指令 (FENCE, FENCE.I, CBO) ============
            0x0F => {
                match funct3 {
                    // FENCE (funct3 = 0)
                    0x0 => {
                        // 内存屏障，当前实现为 NOP
                        self.pc = next_pc;
                    }
                    // FENCE.I (funct3 = 1)
                    0x1 => {
                        // 指令缓存同步，当前实现为 NOP
                        self.pc = next_pc;
                    }
                    // CBO 指令 (funct3 = 2) - Zicbo{m,z,p} 扩展
                    0x2 => {
                        // CBO 指令格式: funct7 决定操作类型
                        // rs1 包含基地址，rs2 用于编码操作类型
                        // 实际上 funct7 = inst[31:25], rs2 = inst[24:20] 但 CBO 使用 rs2 字段编码操作
                        let cbo_op = (inst >> 20) & 0x1F; // rs2 字段作为操作码
                        let va = self.read_reg(rs1);
                        
                        match cbo_op {
                            // cbo.inval (rs2 = 0) - 使缓存行无效
                            0 => {
                                // 由于模拟器没有缓存，实现为 NOP
                                self.pc = next_pc;
                            }
                            // cbo.clean (rs2 = 1) - 清理缓存行（写回）
                            1 => {
                                // 由于模拟器没有缓存，实现为 NOP
                                self.pc = next_pc;
                            }
                            // cbo.flush (rs2 = 2) - 刷新缓存行（清理+无效）
                            2 => {
                                // 由于模拟器没有缓存，实现为 NOP
                                self.pc = next_pc;
                            }
                            // cbo.zero (rs2 = 4) - 将缓存行清零 (Zicboz 扩展)
                            4 => {
                                // 将 64 字节（cache line 大小）清零
                                let satp = self.csr.read(SATP);
                                let mstatus = self.csr.read(MSTATUS);
                                
                                // 对齐到 64 字节边界
                                let aligned_va = va & !63;
                                
                                // 转换虚拟地址并写入 64 字节的零
                                match Mmu::translate(aligned_va, AccessType::Store, self.mode, satp, mstatus, &mut self.bus) {
                                    Ok(pa) => {
                                        // 写入 8 个 u64 的零 (64 字节)
                                        for i in 0..8 {
                                            let _ = self.bus.write64(pa + i * 8, 0);
                                        }
                                        self.pc = next_pc;
                                    }
                                    Err(trap) => {
                                        return Err((trap, aligned_va));
                                    }
                                }
                            }
                            _ => {
                                // 未知的 CBO 操作
                                eprintln!("ERROR: Unknown CBO operation: cbo_op={}, inst=0x{:08x}", cbo_op, inst);
                                return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                            }
                        }
                    }
                    _ => {
                        // 未知的 MISC-MEM 指令
                        eprintln!("ERROR: Unknown MISC-MEM instruction: funct3={}, inst=0x{:08x}", funct3, inst);
                        return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                    }
                }
            }

            // ============ AMO 指令 (原子内存操作，RV64A 扩展) ============
            0x2F => {
                let funct5 = funct7 >> 2;
                let _aq = (funct7 >> 1) & 0x1; // acquire
                let _rl = funct7 & 0x1;        // release
                let va = self.read_reg(rs1);
                
                // 只有 LR.W (funct3=0x2, funct5=0x02) 和 LR.D (funct3=0x3, funct5=0x02) 是纯读操作
                // 其他所有 AMO 指令（SC, AMOSWAP, AMOADD 等）都涉及写操作，
                // 必须使用 AccessType::Store 才能让 MMU 正确设置页表的 Dirty (D) 位
                let access_type = if funct5 == 0x02 {
                    // LR.W / LR.D - Load-Reserved 是读操作
                    AccessType::Load
                } else {
                    // SC 和所有 AMO 运算指令都是写操作
                    AccessType::Store
                };
                
                // 通过 MMU 转换虚拟地址到物理地址
                let satp = self.csr.read(SATP);
                let mstatus = self.csr.read(MSTATUS);
                let pa = Mmu::translate(va, access_type, self.mode, satp, mstatus, &mut self.bus)
                    .map_err(|trap| (trap, va))?;  // 传递 VA 作为 tval

                match (funct3, funct5) {
                    // LR.W (Load-Reserved Word)
                    (0x2, 0x02) => {
                        let value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        self.write_reg(rd, value as i32 as i64 as u64);
                        // 设置 Reservation Set
                        self.set_reservation(pa);
                        self.pc = next_pc;
                    }

                    // SC.W (Store-Conditional Word)
                    (0x2, 0x03) => {
                        if self.check_reservation(pa) {
                            // 保留有效，执行存储
                            let value = self.read_reg(rs2) as u32;
                            self.bus.write32(pa, value)
                                .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                            self.write_reg(rd, 0); // 成功返回 0
                        } else {
                            // 保留无效，存储失败
                            self.write_reg(rd, 1); // 失败返回非零值
                        }
                        // 无论成功与否，SC 指令都会清除保留
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    // AMOSWAP.W (Atomic Swap Word)
                    (0x2, 0x01) => {
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = self.read_reg(rs2) as u32;
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i32 as i64 as u64);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    // AMOADD.W (Atomic Add Word)
                    (0x2, 0x00) => {
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = (old_value as i32).wrapping_add(self.read_reg(rs2) as i32) as u32;
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i32 as i64 as u64);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    // AMOXOR.W, AMOAND.W, AMOOR.W
                    (0x2, 0x04) => {
                        // AMOXOR.W
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value ^ (self.read_reg(rs2) as u32);
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i32 as i64 as u64);
                        // ⚠️ 关键修复：AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x2, 0x0C) => {
                        // AMOAND.W
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value & (self.read_reg(rs2) as u32);
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i32 as i64 as u64);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x2, 0x08) => {
                        // AMOOR.W
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value | (self.read_reg(rs2) as u32);
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i32 as i64 as u64);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    // AMOMIN.W, AMOMAX.W, AMOMINU.W, AMOMAXU.W
                    (0x2, 0x10) => {
                        // AMOMIN.W
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))? as i32;
                        let rs2_value = self.read_reg(rs2) as i32;
                        let new_value = old_value.min(rs2_value) as u32;
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i64 as u64);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x2, 0x14) => {
                        // AMOMAX.W
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))? as i32;
                        let rs2_value = self.read_reg(rs2) as i32;
                        let new_value = old_value.max(rs2_value) as u32;
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i64 as u64);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x2, 0x18) => {
                        // AMOMINU.W
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let rs2_value = self.read_reg(rs2) as u32;
                        let new_value = old_value.min(rs2_value);
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i32 as i64 as u64);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x2, 0x1C) => {
                        // AMOMAXU.W
                        let old_value = self.bus.read32(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let rs2_value = self.read_reg(rs2) as u32;
                        let new_value = old_value.max(rs2_value);
                        self.bus.write32(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value as i32 as i64 as u64);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    // 64-bit AMO instructions (funct3 = 0x3)
                    (0x3, 0x02) => {
                        // LR.D (Load-Reserved Doubleword)
                        let value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        self.write_reg(rd, value);
                        // 设置 Reservation Set
                        self.set_reservation(pa);
                        self.pc = next_pc;
                    }

                    (0x3, 0x03) => {
                        // SC.D (Store-Conditional Doubleword)
                        if self.check_reservation(pa) {
                            // 保留有效，执行存储
                            let value = self.read_reg(rs2);
                            self.bus.write64(pa, value)
                                .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                            self.write_reg(rd, 0); // 成功返回 0
                        } else {
                            // 保留无效，存储失败
                            self.write_reg(rd, 1); // 失败返回非零值
                        }
                        // 无论成功与否，SC 指令都会清除保留
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x01) => {
                        // AMOSWAP.D
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = self.read_reg(rs2);
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        // AMO 指令清除 Reservation Set
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x00) => {
                        // AMOADD.D
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value.wrapping_add(self.read_reg(rs2));
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x04) => {
                        // AMOXOR.D
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value ^ self.read_reg(rs2);
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x08) => {
                        // AMOOR.D - 这就是导致死循环的缺失指令！
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value | self.read_reg(rs2);
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x0C) => {
                        // AMOAND.D
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value & self.read_reg(rs2);
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x10) => {
                        // AMOMIN.D
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = (old_value as i64).min(self.read_reg(rs2) as i64) as u64;
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x14) => {
                        // AMOMAX.D
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = (old_value as i64).max(self.read_reg(rs2) as i64) as u64;
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x18) => {
                        // AMOMINU.D
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value.min(self.read_reg(rs2));
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    (0x3, 0x1C) => {
                        // AMOMAXU.D
                        let old_value = self.bus.read64(pa)
                            .map_err(|_| (Trap::Exception(Exception::LoadAccessFault), va))?;
                        let new_value = old_value.max(self.read_reg(rs2));
                        self.bus.write64(pa, new_value)
                            .map_err(|_| (Trap::Exception(Exception::StoreAMOAccessFault), va))?;
                        self.write_reg(rd, old_value);
                        self.clear_reservation();
                        self.pc = next_pc;
                    }

                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                }
            }

            // ============ 系统指令 (SYSTEM) ============
            0x73 => {
                match funct3 {
                    // ECALL / EBREAK / MRET / SRET / WFI / SFENCE.VMA / HFENCE.GVMA / HFENCE.VVMA
                    0x0 => {
                        // 检查是否为 SFENCE.VMA (funct7 = 0x09)
                        if funct7 == 0x09 {
                            // SFENCE.VMA - 刷新 TLB (Translation Lookaside Buffer)
                            // 由于当前实现没有 TLB 缓存（每次访问都查页表），
                            // 因此实现为 No-Op（空操作），这是符合 RISC-V 规范的。
                            self.pc = next_pc;
                        } else if funct7 == 0x31 {
                            // HFENCE.GVMA (0x62000073) - Hypervisor 扩展的 Guest Physical Address TLB Flush
                            // funct7 = 0110001 (0x31)
                            // 用于刷新虚拟机的 GPA -> HPA 映射缓存
                            // 当前实现无 TLB，实现为 No-Op
                            self.pc = next_pc;
                        } else if funct7 == 0x11 {
                            // HFENCE.VVMA (0x22000073) - Hypervisor 扩展的 Virtual Virtual Address TLB Flush
                            // funct7 = 0010001 (0x11)
                            // 用于刷新虚拟机的 VA -> GPA 映射缓存
                            // 当前实现无 TLB，实现为 No-Op
                            self.pc = next_pc;
                        } else {
                            match imm_i {
                                0x000 => {
                                    // ECALL - 根据当前特权模式返回不同的异常
                                    match self.mode {
                                        Mode::User => {
                                            return Err((Trap::Exception(Exception::EnvironmentCallFromUMode), 0));
                                        }
                                        Mode::Supervisor => return Err((Trap::Exception(Exception::EnvironmentCallFromSMode), 0)),
                                        Mode::Machine | Mode::Hypervisor => return Err((Trap::Exception(Exception::EnvironmentCallFromMMode), 0)),
                                    }
                                }
                                0x001 => {
                                    // EBREAK - tval 应该是触发 EBREAK 的地址（PC）
                                    return Err((Trap::Exception(Exception::Breakpoint), self.pc));
                                }
                                0x302 => {
                                    // MRET (Machine-mode Return)
                                    let mut mstatus = self.csr.read(MSTATUS);
                                    let mpp = (mstatus >> 11) & 0x3;
                                    let mpie = (mstatus >> 7) & 0x1;

                                    // MIE <- MPIE
                                    mstatus = (mstatus & !(1 << 3)) | (mpie << 3);
                                    // MPIE <- 1
                                    mstatus |= 1 << 7;
                                    // MPP <- 0
                                    mstatus &= !(0x3 << 11);
                                    self.csr.write(MSTATUS, mstatus);

                                    self.mode = match mpp {
                                        0 => Mode::User,
                                        1 => Mode::Supervisor,
                                        2 => Mode::Hypervisor,
                                        _ => Mode::Machine,
                                    };
                                    self.pc = self.csr.read(MEPC);
                                    // MRET 清除 Reservation Set
                                    self.clear_reservation();
                                }
                                0x102 => {
                                    // SRET (Supervisor-mode Return)
                                    let mut sstatus = self.csr.read(SSTATUS);
                                    let spp = (sstatus >> 8) & 0x1;
                                    let spie = (sstatus >> 5) & 0x1;

                                    // SIE <- SPIE
                                    sstatus = (sstatus & !(1 << 1)) | (spie << 1);
                                    // SPIE <- 1
                                    sstatus |= 1 << 5;
                                    // SPP <- 0
                                    sstatus &= !(1 << 8);
                                    self.csr.write(SSTATUS, sstatus);

                                    self.mode = if spp == 0 { Mode::User } else { Mode::Supervisor };
                                    self.pc = self.csr.read(SEPC);
                                    // SRET 清除 Reservation Set
                                    self.clear_reservation();
                                }
                                0x105 => {
                                    // WFI (Wait for Interrupt) - 简单实现为 NOP
                                    self.pc = next_pc;
                                }
                                0x00D => {
                                    // WRS.NTO (Wait-on-Reservation-Set, No Time-out) - Zawrs 扩展
                                    // 如果没有活动的 reservation，可以暂停执行等待
                                    // 简单实现：当作 NOP 处理
                                    self.pc = next_pc;
                                }
                                0x01D => {
                                    // WRS.STO (Wait-on-Reservation-Set, Some Time-out) - Zawrs 扩展
                                    // 类似 WRS.NTO，但有超时限制
                                    // 简单实现：当作 NOP 处理
                                    self.pc = next_pc;
                                }
                                _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                            }
                        }
                    }

                    // CSRRW
                    0x1 => {
                        let csr_addr = (inst >> 20) as u16;
                        // 检查 CSR 地址是否合法
                        if !Csr::is_valid_csr(csr_addr) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        // 检查 CSR 特权级
                        // CSRRW 总是会写入，所以 is_write=true
                        if !Csr::check_csr_privilege(csr_addr, self.mode as u8, true) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        // 拦截 STIMECMP (0x14D) 写操作，同步到 Bus
                        if csr_addr == 0x14D {
                            let old_val = self.bus.stimecmp;
                            let new_val = if rs1 == 0 { 0 } else { self.read_reg(rs1) };
                            
                            // 写入 Bus (让硬件逻辑生效)
                            self.bus.stimecmp = new_val;
                            // 同时写入 CSR 结构体 (保持状态一致)
                            self.csr.write(csr_addr, new_val);
                            // CSRRW 返回旧值
                            self.write_reg(rd, old_val);
                            
                            self.pc = next_pc;
                            return Ok(());
                        }
                        
                        // 拦截 CSR time (0xC01) 读取，从 CLINT 获取 mtime
                        let old_value = if csr_addr == crate::csr::TIME {
                            // 从 CLINT mtime 寄存器读取当前时间
                            self.bus.read64(crate::bus::CLINT_BASE + 0xBFF8)
                                .unwrap_or(0)
                        } else {
                            let value = self.read_reg(rs1);
                            self.csr.csrrw(csr_addr, value)
                        };
                        
                        self.write_reg(rd, old_value);
                        self.pc = next_pc;
                    }

                    // CSRRS
                    0x2 => {
                        let csr_addr = (inst >> 20) as u16;
                        // 检查 CSR 地址是否合法
                        if !Csr::is_valid_csr(csr_addr) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        // 检查 CSR 特权级
                        // CSRRS 只有当 rs1 != 0 时才会写入
                        let is_write = rs1 != 0;
                        // 当 rd != 0 或 rs1 != 0 时需要进行读访问（含读副作用）
                        let should_read = rd != 0 || rs1 != 0;
                        if !Csr::check_csr_privilege(csr_addr, self.mode as u8, is_write) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        
                        // 拦截 CSR time (0xC01) 读取，从 CLINT 获取 mtime
                        let old_value = if !should_read {
                            0
                        } else if csr_addr == crate::csr::TIME {
                            // 从 CLINT mtime 寄存器读取当前时间
                            self.bus.read64(crate::bus::CLINT_BASE + 0xBFF8)
                                .unwrap_or(0)
                        } else {
                            let mask = self.read_reg(rs1);
                            self.csr.csrrs(csr_addr, mask)
                        };
                        
                        // STIMECMP 写入时同步到 Bus
                        if is_write && csr_addr == STIMECMP {
                            let new_val = self.csr.read(STIMECMP);
                            self.bus.stimecmp = new_val;
                        }
                        
                        self.write_reg(rd, old_value);
                        self.pc = next_pc;
                    }

                    // CSRRC
                    0x3 => {
                        let csr_addr = (inst >> 20) as u16;
                        // 检查 CSR 地址是否合法
                        if !Csr::is_valid_csr(csr_addr) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        // 检查 CSR 特权级
                        // CSRRC 只有当 rs1 != 0 时才会写入
                        let is_write = rs1 != 0;
                        // 当 rd != 0 或 rs1 != 0 时需要进行读访问（含读副作用）
                        let should_read = rd != 0 || rs1 != 0;
                        if !Csr::check_csr_privilege(csr_addr, self.mode as u8, is_write) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        
                        // 拦截 CSR time (0xC01) 读取，从 CLINT 获取 mtime
                        let old_value = if !should_read {
                            0
                        } else if csr_addr == crate::csr::TIME {
                            // 从 CLINT mtime 寄存器读取当前时间
                            self.bus.read64(crate::bus::CLINT_BASE + 0xBFF8)
                                .unwrap_or(0)
                        } else {
                            let mask = self.read_reg(rs1);
                            self.csr.csrrc(csr_addr, mask)
                        };
                        
                        //STIMECMP 写入时同步到 Bus
                        if is_write && csr_addr == STIMECMP {
                            let new_val = self.csr.read(STIMECMP);
                            self.bus.stimecmp = new_val;
                        }
                        
                        self.write_reg(rd, old_value);
                        self.pc = next_pc;
                    }

                    // CSRRWI
                    0x5 => {
                        let csr_addr = (inst >> 20) as u16;
                        // 检查 CSR 地址是否合法
                        if !Csr::is_valid_csr(csr_addr) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        // 检查 CSR 特权级
                        // CSRRWI 总是会写入，所以 is_write=true
                        if !Csr::check_csr_privilege(csr_addr, self.mode as u8, true) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        
                        //  拦截 STIMECMP (0x14D) 写操作，同步到 Bus
                        if csr_addr == STIMECMP {
                            let imm = rs1 as u64; // zimm[4:0]
                            let old_val = self.csr.csrrw(csr_addr, imm);
                            self.bus.stimecmp = imm;
                            self.write_reg(rd, old_val);
                            self.pc = next_pc;
                            return Ok(());
                        }
                        
                        // 拦截 CSR time (0xC01) 读取，从 CLINT 获取 mtime
                        let old_value = if csr_addr == crate::csr::TIME {
                            // 从 CLINT mtime 寄存器读取当前时间
                            self.bus.read64(crate::bus::CLINT_BASE + 0xBFF8)
                                .unwrap_or(0)
                        } else {
                            let imm = rs1 as u64; // zimm[4:0]
                            self.csr.csrrw(csr_addr, imm)
                        };
                        
                        self.write_reg(rd, old_value);
                        self.pc = next_pc;
                    }

                    // CSRRSI
                    0x6 => {
                        let csr_addr = (inst >> 20) as u16;
                        // 检查 CSR 地址是否合法
                        if !Csr::is_valid_csr(csr_addr) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        // 检查 CSR 特权级
                        // CSRRSI 只有当 zimm != 0 时才会写入
                        let is_write = rs1 != 0;  // rs1 字段用作 zimm
                        // 当 rd != 0 或 zimm != 0 时需要进行读访问（含读副作用）
                        let should_read = rd != 0 || rs1 != 0;
                        if !Csr::check_csr_privilege(csr_addr, self.mode as u8, is_write) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        
                        // 拦截 CSR time (0xC01) 读取，从 CLINT 获取 mtime
                        let old_value = if !should_read {
                            0
                        } else if csr_addr == crate::csr::TIME {
                            // 从 CLINT mtime 寄存器读取当前时间
                            self.bus.read64(crate::bus::CLINT_BASE + 0xBFF8)
                                .unwrap_or(0)
                        } else {
                            let imm = rs1 as u64;
                            self.csr.csrrs(csr_addr, imm)
                        };
                        
                        // [BUG修复] STIMECMP 写入时同步到 Bus
                        if is_write && csr_addr == STIMECMP {
                            let new_val = self.csr.read(STIMECMP);
                            self.bus.stimecmp = new_val;
                        }
                        
                        self.write_reg(rd, old_value);
                        self.pc = next_pc;
                    }

                    // CSRRCI
                    0x7 => {
                        let csr_addr = (inst >> 20) as u16;
                        // 检查 CSR 地址是否合法
                        if !Csr::is_valid_csr(csr_addr) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        // 检查 CSR 特权级
                        // CSRRCI 只有当 zimm != 0 时才会写入
                        let is_write = rs1 != 0;  // rs1 字段用作 zimm
                        // 当 rd != 0 或 zimm != 0 时需要进行读访问（含读副作用）
                        let should_read = rd != 0 || rs1 != 0;
                        if !Csr::check_csr_privilege(csr_addr, self.mode as u8, is_write) {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        
                        // 拦截 CSR time (0xC01) 读取，从 CLINT 获取 mtime
                        let old_value = if !should_read {
                            0
                        } else if csr_addr == crate::csr::TIME {
                            // 从 CLINT mtime 寄存器读取当前时间
                            self.bus.read64(crate::bus::CLINT_BASE + 0xBFF8)
                                .unwrap_or(0)
                        } else {
                            let imm = rs1 as u64;
                            self.csr.csrrc(csr_addr, imm)
                        };
                        
                        // [BUG修复] STIMECMP 写入时同步到 Bus
                        if is_write && csr_addr == STIMECMP {
                            let new_val = self.csr.read(STIMECMP);
                            self.bus.stimecmp = new_val;
                        }
                        
                        self.write_reg(rd, old_value);
                        self.pc = next_pc;
                    }

                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                }
            }

            // ============ 浮点指令 (OP-FP) ============
            0x53 => {
                // 检查 mstatus.FS 是否启用
                let mstatus = self.csr.read(MSTATUS);
                let fs = (mstatus >> 13) & 0x3;
                if fs == 0 {
                    // FS = Off，浮点指令不可用
                    return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                }

                // 辅助宏：设置 FS = Dirty
                macro_rules! set_fs_dirty {
                    () => {{
                        let new_mstatus = (mstatus & !(0x3 << 13)) | (0x3 << 13);
                        self.csr.write(MSTATUS, new_mstatus);
                    }};
                }

                // 辅助函数：检查单精度 NaN-boxing，返回 f32
                fn unbox_f32(val: u64) -> f32 {
                    if (val >> 32) == 0xFFFF_FFFF {
                        f32::from_bits(val as u32)
                    } else {
                        f32::from_bits(0x7FC0_0000) // canonical NaN
                    }
                }

                // 辅助函数：NaN-box f32 到 f64 寄存器
                fn box_f32(val: f32) -> u64 {
                    0xFFFF_FFFF_0000_0000 | (val.to_bits() as u64)
                }

                match funct7 {
                    // ============ 双精度算术运算 ============
                    0x01 => {
                        // FADD.D
                        let a = f64::from_bits(self.f_regs[rs1]);
                        let b = f64::from_bits(self.f_regs[rs2]);
                        self.f_regs[rd] = (a + b).to_bits();
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }
                    0x05 => {
                        // FSUB.D
                        let a = f64::from_bits(self.f_regs[rs1]);
                        let b = f64::from_bits(self.f_regs[rs2]);
                        self.f_regs[rd] = (a - b).to_bits();
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }
                    0x09 => {
                        // FMUL.D
                        let a = f64::from_bits(self.f_regs[rs1]);
                        let b = f64::from_bits(self.f_regs[rs2]);
                        self.f_regs[rd] = (a * b).to_bits();
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }
                    0x0D => {
                        // FDIV.D
                        let a = f64::from_bits(self.f_regs[rs1]);
                        let b = f64::from_bits(self.f_regs[rs2]);
                        self.f_regs[rd] = (a / b).to_bits();
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }
                    0x2D => {
                        // FSQRT.D (rs2 must be 0)
                        if rs2 != 0 {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        let a = f64::from_bits(self.f_regs[rs1]);
                        self.f_regs[rd] = a.sqrt().to_bits();
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }

                    // ============ 单精度算术运算 ============
                    0x00 => {
                        // FADD.S
                        let a = unbox_f32(self.f_regs[rs1]);
                        let b = unbox_f32(self.f_regs[rs2]);
                        self.f_regs[rd] = box_f32(a + b);
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }
                    0x04 => {
                        // FSUB.S
                        let a = unbox_f32(self.f_regs[rs1]);
                        let b = unbox_f32(self.f_regs[rs2]);
                        self.f_regs[rd] = box_f32(a - b);
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }
                    0x08 => {
                        // FMUL.S
                        let a = unbox_f32(self.f_regs[rs1]);
                        let b = unbox_f32(self.f_regs[rs2]);
                        self.f_regs[rd] = box_f32(a * b);
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }
                    0x0C => {
                        // FDIV.S
                        let a = unbox_f32(self.f_regs[rs1]);
                        let b = unbox_f32(self.f_regs[rs2]);
                        self.f_regs[rd] = box_f32(a / b);
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }
                    0x2C => {
                        // FSQRT.S (rs2 must be 0)
                        if rs2 != 0 {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        let a = unbox_f32(self.f_regs[rs1]);
                        self.f_regs[rd] = box_f32(a.sqrt());
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }

                    // ============ 双精度符号注入 ============
                    0x11 => {
                        // FSGNJ.D / FSGNJN.D / FSGNJX.D
                        let a = self.f_regs[rs1];
                        let b = self.f_regs[rs2];
                        let result = match funct3 {
                            0x0 => (a & 0x7FFF_FFFF_FFFF_FFFF) | (b & 0x8000_0000_0000_0000), // FSGNJ
                            0x1 => (a & 0x7FFF_FFFF_FFFF_FFFF) | ((b ^ 0x8000_0000_0000_0000) & 0x8000_0000_0000_0000), // FSGNJN
                            0x2 => a ^ (b & 0x8000_0000_0000_0000), // FSGNJX
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.f_regs[rd] = result;
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }

                    // ============ 单精度符号注入 ============
                    0x10 => {
                        // FSGNJ.S / FSGNJN.S / FSGNJX.S
                        let a = self.f_regs[rs1] as u32;
                        let b = self.f_regs[rs2] as u32;
                        let result = match funct3 {
                            0x0 => (a & 0x7FFF_FFFF) | (b & 0x8000_0000), // FSGNJ
                            0x1 => (a & 0x7FFF_FFFF) | ((b ^ 0x8000_0000) & 0x8000_0000), // FSGNJN
                            0x2 => a ^ (b & 0x8000_0000), // FSGNJX
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.f_regs[rd] = 0xFFFF_FFFF_0000_0000 | (result as u64);
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }

                    // ============ 双精度最小/最大 ============
                    0x15 => {
                        // FMIN.D / FMAX.D (符合 RISC-V 规范的 NaN 和 -0.0 处理)
                        let a = f64::from_bits(self.f_regs[rs1]);
                        let b = f64::from_bits(self.f_regs[rs2]);
                        let result = match funct3 {
                            0x0 => {
                                // FMIN.D: 如果一个是 NaN，返回另一个；-0 < +0
                                if a.is_nan() && b.is_nan() {
                                    f64::from_bits(0x7FF8_0000_0000_0000) // canonical NaN
                                } else if a.is_nan() {
                                    b
                                } else if b.is_nan() {
                                    a
                                } else if a == 0.0 && b == 0.0 {
                                    // -0.0 < +0.0
                                    if a.is_sign_negative() { a } else { b }
                                } else {
                                    a.min(b)
                                }
                            }
                            0x1 => {
                                // FMAX.D: 如果一个是 NaN，返回另一个；+0 > -0
                                if a.is_nan() && b.is_nan() {
                                    f64::from_bits(0x7FF8_0000_0000_0000) // canonical NaN
                                } else if a.is_nan() {
                                    b
                                } else if b.is_nan() {
                                    a
                                } else if a == 0.0 && b == 0.0 {
                                    // +0.0 > -0.0
                                    if a.is_sign_positive() { a } else { b }
                                } else {
                                    a.max(b)
                                }
                            }
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.f_regs[rd] = result.to_bits();
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }

                    // ============ 单精度最小/最大 ============
                    0x14 => {
                        // FMIN.S / FMAX.S (符合 RISC-V 规范的 NaN 和 -0.0 处理)
                        let a = unbox_f32(self.f_regs[rs1]);
                        let b = unbox_f32(self.f_regs[rs2]);
                        let result = match funct3 {
                            0x0 => {
                                // FMIN.S
                                if a.is_nan() && b.is_nan() {
                                    f32::from_bits(0x7FC0_0000) // canonical NaN
                                } else if a.is_nan() {
                                    b
                                } else if b.is_nan() {
                                    a
                                } else if a == 0.0 && b == 0.0 {
                                    if a.is_sign_negative() { a } else { b }
                                } else {
                                    a.min(b)
                                }
                            }
                            0x1 => {
                                // FMAX.S
                                if a.is_nan() && b.is_nan() {
                                    f32::from_bits(0x7FC0_0000) // canonical NaN
                                } else if a.is_nan() {
                                    b
                                } else if b.is_nan() {
                                    a
                                } else if a == 0.0 && b == 0.0 {
                                    if a.is_sign_positive() { a } else { b }
                                } else {
                                    a.max(b)
                                }
                            }
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.f_regs[rd] = box_f32(result);
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }

                    // ============ 双精度比较 ============
                    0x51 => {
                        // FEQ.D / FLT.D / FLE.D (结果写入整数寄存器)
                        let a = f64::from_bits(self.f_regs[rs1]);
                        let b = f64::from_bits(self.f_regs[rs2]);
                        let result = match funct3 {
                            0x2 => if a == b { 1u64 } else { 0u64 }, // FEQ.D
                            0x1 => if a < b { 1u64 } else { 0u64 },  // FLT.D
                            0x0 => if a <= b { 1u64 } else { 0u64 }, // FLE.D
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.write_reg(rd, result);
                        self.pc = next_pc;
                    }

                    // ============ 单精度比较 ============
                    0x50 => {
                        // FEQ.S / FLT.S / FLE.S (结果写入整数寄存器)
                        let a = unbox_f32(self.f_regs[rs1]);
                        let b = unbox_f32(self.f_regs[rs2]);
                        let result = match funct3 {
                            0x2 => if a == b { 1u64 } else { 0u64 }, // FEQ.S
                            0x1 => if a < b { 1u64 } else { 0u64 },  // FLT.S
                            0x0 => if a <= b { 1u64 } else { 0u64 }, // FLE.S
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.write_reg(rd, result);
                        self.pc = next_pc;
                    }

                    // ============ 双精度转换 (浮点 -> 整数) ============
                    0x61 => {
                        // FCVT.W.D / FCVT.WU.D / FCVT.L.D / FCVT.LU.D
                        let a = f64::from_bits(self.f_regs[rs1]);
                        let result = match rs2 {
                            0 => (a as i32) as i64 as u64,   // FCVT.W.D (符号扩展)
                            1 => (a as u32) as u64,          // FCVT.WU.D
                            2 => (a as i64) as u64,          // FCVT.L.D
                            3 => a as u64,                   // FCVT.LU.D
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.write_reg(rd, result);
                        self.pc = next_pc;
                    }

                    // ============ 双精度转换 (整数 -> 浮点) ============
                    0x69 => {
                        // FCVT.D.W / FCVT.D.WU / FCVT.D.L / FCVT.D.LU
                        let val = self.read_reg(rs1);
                        let result = match rs2 {
                            0 => (val as i32 as f64).to_bits(),   // FCVT.D.W
                            1 => (val as u32 as f64).to_bits(),   // FCVT.D.WU
                            2 => (val as i64 as f64).to_bits(),   // FCVT.D.L
                            3 => (val as u64 as f64).to_bits(),   // FCVT.D.LU
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.f_regs[rd] = result;
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }

                    // ============ 单精度转换 (浮点 -> 整数) ============
                    0x60 => {
                        // FCVT.W.S / FCVT.WU.S / FCVT.L.S / FCVT.LU.S
                        let a = unbox_f32(self.f_regs[rs1]);
                        let result = match rs2 {
                            0 => (a as i32) as i64 as u64,   // FCVT.W.S
                            1 => (a as u32) as u64,          // FCVT.WU.S
                            2 => (a as i64) as u64,          // FCVT.L.S
                            3 => a as u64,                   // FCVT.LU.S
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.write_reg(rd, result);
                        self.pc = next_pc;
                    }

                    // ============ 单精度转换 (整数 -> 浮点) ============
                    0x68 => {
                        // FCVT.S.W / FCVT.S.WU / FCVT.S.L / FCVT.S.LU
                        let val = self.read_reg(rs1);
                        let result = match rs2 {
                            0 => box_f32(val as i32 as f32),   // FCVT.S.W
                            1 => box_f32(val as u32 as f32),   // FCVT.S.WU
                            2 => box_f32(val as i64 as f32),   // FCVT.S.L
                            3 => box_f32(val as u64 as f32),   // FCVT.S.LU
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        };
                        self.f_regs[rd] = result;
                        set_fs_dirty!();
                        self.pc = next_pc;
                    }

                    // ============ 单精度 <-> 双精度转换 ============
                    0x20 => {
                        // FCVT.S.D (rs2=1) / FCVT.S.H (rs2=2, 暂不支持)
                        if rs2 == 1 {
                            // FCVT.S.D: 双精度 -> 单精度
                            let a = f64::from_bits(self.f_regs[rs1]);
                            self.f_regs[rd] = box_f32(a as f32);
                            set_fs_dirty!();
                            self.pc = next_pc;
                        } else {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                    }
                    0x21 => {
                        // FCVT.D.S (rs2=0) / FCVT.D.H (rs2=2, 暂不支持)
                        if rs2 == 0 {
                            // FCVT.D.S: 单精度 -> 双精度
                            let a = unbox_f32(self.f_regs[rs1]);
                            self.f_regs[rd] = (a as f64).to_bits();
                            set_fs_dirty!();
                            self.pc = next_pc;
                        } else {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                    }

                    // ============ 分类指令 ============
                    0x71 => {
                        // FMV.X.D (funct3=0, rs2=0) 或 FCLASS.D (funct3=1, rs2=0)
                        if rs2 != 0 {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        match funct3 {
                            0x0 => {
                                // FMV.X.D
                                self.write_reg(rd, self.f_regs[rs1]);
                                self.pc = next_pc;
                            }
                            0x1 => {
                                // FCLASS.D
                                let val = self.f_regs[rs1];
                                let f = f64::from_bits(val);
                                let class = if f.is_nan() {
                                    if (val & 0x0008_0000_0000_0000) != 0 { 1 << 9 } // quiet NaN
                                    else { 1 << 8 } // signaling NaN
                                } else if f.is_infinite() {
                                    if f.is_sign_positive() { 1 << 7 } else { 1 << 0 }
                                } else if f == 0.0 {
                                    if f.is_sign_positive() { 1 << 4 } else { 1 << 3 }
                                } else if f.is_subnormal() {
                                    if f.is_sign_positive() { 1 << 5 } else { 1 << 2 }
                                } else {
                                    if f.is_sign_positive() { 1 << 6 } else { 1 << 1 }
                                };
                                self.write_reg(rd, class);
                                self.pc = next_pc;
                            }
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        }
                    }

                    0x70 => {
                        // FMV.X.W (funct3=0, rs2=0) 或 FCLASS.S (funct3=1, rs2=0)
                        if rs2 != 0 {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                        match funct3 {
                            0x0 => {
                                // FMV.X.W
                                let value = self.f_regs[rs1] as i32 as i64 as u64;
                                self.write_reg(rd, value);
                                self.pc = next_pc;
                            }
                            0x1 => {
                                // FCLASS.S
                                let f = unbox_f32(self.f_regs[rs1]);
                                let val = f.to_bits();
                                let class = if f.is_nan() {
                                    if (val & 0x0040_0000) != 0 { 1 << 9 } // quiet NaN
                                    else { 1 << 8 } // signaling NaN
                                } else if f.is_infinite() {
                                    if f.is_sign_positive() { 1 << 7 } else { 1 << 0 }
                                } else if f == 0.0 {
                                    if f.is_sign_positive() { 1 << 4 } else { 1 << 3 }
                                } else if f.is_subnormal() {
                                    if f.is_sign_positive() { 1 << 5 } else { 1 << 2 }
                                } else {
                                    if f.is_sign_positive() { 1 << 6 } else { 1 << 1 }
                                };
                                self.write_reg(rd, class as u64);
                                self.pc = next_pc;
                            }
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        }
                    }

                    // ============ 移动指令 ============
                    0x78 => {
                        // FMV.W.X
                        if funct3 == 0x0 && rs2 == 0x0 {
                            let value = self.read_reg(rs1) as u32;
                            self.f_regs[rd] = 0xFFFF_FFFF_0000_0000 | (value as u64);
                            set_fs_dirty!();
                            self.pc = next_pc;
                        } else {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                    }
                    0x79 => {
                        // FMV.D.X
                        if funct3 == 0x0 && rs2 == 0x0 {
                            self.f_regs[rd] = self.read_reg(rs1);
                            set_fs_dirty!();
                            self.pc = next_pc;
                        } else {
                            return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                        }
                    }

                    _ => {
                        // 其他浮点指令暂不支持
                        eprintln!("ERROR: Unimplemented FP instruction: funct7=0x{:02x}, funct3=0x{:x}", funct7, funct3);
                        return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                    }
                }
            }

            // ============ 浮点融合乘加指令 (FMADD/FMSUB/FNMSUB/FNMADD) ============
            0x43 | 0x47 | 0x4B | 0x4F => {
                // 检查 mstatus.FS 是否启用
                let mstatus = self.csr.read(MSTATUS);
                let fs = (mstatus >> 13) & 0x3;
                if fs == 0 {
                    return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                }

                let rs3 = ((inst >> 27) & 0x1F) as usize;
                let fmt = (inst >> 25) & 0x3; // 00=S, 01=D

                // 辅助函数
                fn unbox_f32(val: u64) -> f32 {
                    if (val >> 32) == 0xFFFF_FFFF {
                        f32::from_bits(val as u32)
                    } else {
                        f32::from_bits(0x7FC0_0000)
                    }
                }
                fn box_f32(val: f32) -> u64 {
                    0xFFFF_FFFF_0000_0000 | (val.to_bits() as u64)
                }

                match fmt {
                    0x00 => {
                        // 单精度
                        let a = unbox_f32(self.f_regs[rs1]);
                        let b = unbox_f32(self.f_regs[rs2]);
                        let c = unbox_f32(self.f_regs[rs3]);
                        let result = match opcode {
                            0x43 => a.mul_add(b, c),           // FMADD.S:  a*b + c
                            0x47 => a.mul_add(b, -c),          // FMSUB.S:  a*b - c
                            0x4B => (-a).mul_add(b, c),        // FNMSUB.S: -a*b + c
                            0x4F => (-a).mul_add(b, -c),       // FNMADD.S: -a*b - c
                            _ => unreachable!(),
                        };
                        self.f_regs[rd] = box_f32(result);
                    }
                    0x01 => {
                        // 双精度
                        let a = f64::from_bits(self.f_regs[rs1]);
                        let b = f64::from_bits(self.f_regs[rs2]);
                        let c = f64::from_bits(self.f_regs[rs3]);
                        let result = match opcode {
                            0x43 => a.mul_add(b, c),           // FMADD.D
                            0x47 => a.mul_add(b, -c),          // FMSUB.D
                            0x4B => (-a).mul_add(b, c),        // FNMSUB.D
                            0x4F => (-a).mul_add(b, -c),       // FNMADD.D
                            _ => unreachable!(),
                        };
                        self.f_regs[rd] = result.to_bits();
                    }
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                }

                // 设置 FS = Dirty
                let new_mstatus = (mstatus & !(0x3 << 13)) | (0x3 << 13);
                self.csr.write(MSTATUS, new_mstatus);
                self.pc = next_pc;
            }

            // ============ 未实现的指令 ============
            _ => {
                return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
            }
        }

        Ok(())
    }

    /// ============ 压缩指令执行 (RVC 扩展) ============
    fn execute_compressed(&mut self, inst: u16) -> Result<(), (Trap, u64)> {
        let opcode = inst & 0x3;        // bits [1:0]
        let funct3 = (inst >> 13) & 0x7; // bits [15:13]
        
        let next_pc = self.pc.wrapping_add(2);

        // 临时调试：打印未匹配的指令
        // eprintln!("DEBUG: Compressed inst=0x{:04x}, opcode={}, funct3={}", inst, opcode, funct3);

        match (opcode, funct3) {
            // ============ C0 (opcode = 00) ============
            (0b00, 0b000) => {
                // C.ADDI4SPN
                let rd_p = ((inst >> 2) & 0x7) as usize + 8;
                let imm = (((inst >> 11) & 0x3) << 4)
                    | (((inst >> 7) & 0xF) << 6)
                    | (((inst >> 6) & 0x1) << 2)
                    | (((inst >> 5) & 0x1) << 3);
                if imm == 0 {
                    return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                }
                let value = self.read_reg(2).wrapping_add(imm as u64);
                self.write_reg(rd_p, value);
                self.pc = next_pc;
            }

            (0b00, 0b001) => {
                // C.FLD - 压缩浮点双精度加载 (RV32DC/RV64DC)
                let rd_p = ((inst >> 2) & 0x7) as usize + 8;
                let rs1_p = ((inst >> 7) & 0x7) as usize + 8;
                let imm = (((inst >> 10) & 0x7) << 3) | (((inst >> 5) & 0x3) << 6);
                let va = self.read_reg(rs1_p).wrapping_add(imm as u64);
                
                let value = self.load_va(va, 8)?;
                self.f_regs[rd_p] = value;
                
                // 设置 mstatus.FS = Dirty (3)
                let mstatus = self.csr.read(MSTATUS);
                let new_mstatus = (mstatus & !(0x3 << 13)) | (0x3 << 13);
                self.csr.write(MSTATUS, new_mstatus);
                
                self.pc = next_pc;
            }

            (0b00, 0b010) => {
                // C.LW
                let rd_p = ((inst >> 2) & 0x7) as usize + 8;
                let rs1_p = ((inst >> 7) & 0x7) as usize + 8;
                let imm = (((inst >> 10) & 0x7) << 3)
                    | (((inst >> 6) & 0x1) << 2)
                    | (((inst >> 5) & 0x1) << 6);
                let va = self.read_reg(rs1_p).wrapping_add(imm as u64);
                
                let value = self.load_va(va, 4)?;
                self.write_reg(rd_p, (value as u32) as i32 as i64 as u64);
                self.pc = next_pc;
            }

            (0b00, 0b011) => {
                // C.LD (RV64)
                let rd_p = ((inst >> 2) & 0x7) as usize + 8;
                let rs1_p = ((inst >> 7) & 0x7) as usize + 8;
                let imm = (((inst >> 10) & 0x7) << 3) | (((inst >> 5) & 0x3) << 6);
                let va = self.read_reg(rs1_p).wrapping_add(imm as u64);
                
                let value = self.load_va(va, 8)?;
                self.write_reg(rd_p, value);
                self.pc = next_pc;
            }

            (0b00, 0b101) => {
                // C.FSD - 压缩浮点双精度存储 (RV32DC/RV64DC)
                let rs2_p = ((inst >> 2) & 0x7) as usize + 8;
                let rs1_p = ((inst >> 7) & 0x7) as usize + 8;
                let imm = (((inst >> 10) & 0x7) << 3) | (((inst >> 5) & 0x3) << 6);
                let va = self.read_reg(rs1_p).wrapping_add(imm as u64);
                
                let value = self.f_regs[rs2_p];
                self.store_va(va, value, 8)?;
                self.clear_reservation();
                self.pc = next_pc;
            }

            (0b00, 0b110) => {
                // C.SW
                let rs2_p = ((inst >> 2) & 0x7) as usize + 8;
                let rs1_p = ((inst >> 7) & 0x7) as usize + 8;
                let imm = (((inst >> 10) & 0x7) << 3)
                    | (((inst >> 6) & 0x1) << 2)
                    | (((inst >> 5) & 0x1) << 6);
                let va = self.read_reg(rs1_p).wrapping_add(imm as u64);
                let value = self.read_reg(rs2_p);
                
                self.store_va(va, value, 4)?;
                self.clear_reservation();
                self.pc = next_pc;
            }

            (0b00, 0b111) => {
                // C.SD
                let rs2_p = ((inst >> 2) & 0x7) as usize + 8;
                let rs1_p = ((inst >> 7) & 0x7) as usize + 8;
                let imm = (((inst >> 10) & 0x7) << 3) | (((inst >> 5) & 0x3) << 6);
                let va = self.read_reg(rs1_p).wrapping_add(imm as u64);
                let value = self.read_reg(rs2_p);
                
                self.store_va(va, value, 8)?;
                self.clear_reservation();
                self.pc = next_pc;
            }

            // ============ C1 (opcode = 01) ============
            (0b01, 0b000) => {
                // C.ADDI (非 NOP)
                let rd = ((inst >> 7) & 0x1F) as usize;
                if rd == 0 {
                    // C.NOP
                    self.pc = next_pc;
                } else {
                    let imm = (((inst >> 12) & 0x1) << 5) | ((inst >> 2) & 0x1F);
                    let imm = ((imm as i32) << 26 >> 26) as i64 as u64; // 符号扩展
                    let value = self.read_reg(rd).wrapping_add(imm);
                    self.write_reg(rd, value);
                    self.pc = next_pc;
                }
            }

            (0b01, 0b001) => {
                // C.ADDIW (RV64)
                let rd = ((inst >> 7) & 0x1F) as usize;
                let imm = (((inst >> 12) & 0x1) << 5) | ((inst >> 2) & 0x1F);
                let imm = ((imm as i32) << 26 >> 26) as i64; // 符号扩展
                let value = (self.read_reg(rd) as i32).wrapping_add(imm as i32);
                self.write_reg(rd, value as i64 as u64);
                self.pc = next_pc;
            }

            (0b01, 0b010) => {
                // C.LI
                let rd = ((inst >> 7) & 0x1F) as usize;
                let imm = (((inst >> 12) & 0x1) << 5) | ((inst >> 2) & 0x1F);
                let imm = ((imm as i32) << 26 >> 26) as i64 as u64; // 符号扩展
                self.write_reg(rd, imm);
                self.pc = next_pc;
            }

            (0b01, 0b011) => {
                let rd = ((inst >> 7) & 0x1F) as usize;
                if rd == 2 {
                    // C.ADDI16SP
                    let imm = (((inst >> 12) & 0x1) << 9)
                        | (((inst >> 6) & 0x1) << 4)
                        | (((inst >> 5) & 0x1) << 6)
                        | (((inst >> 3) & 0x3) << 7)
                        | (((inst >> 2) & 0x1) << 5);
                    let imm = ((imm as i32) << 22 >> 22) as i64 as u64;
                    let value = self.read_reg(2).wrapping_add(imm);
                    self.write_reg(2, value);
                    self.pc = next_pc;
                } else {
                    // C.LUI
                    let imm = ((((inst as u32) >> 12) & 0x1) << 17) | ((((inst as u32) >> 2) & 0x1F) << 12);
                    let imm = ((imm as i32) << 14 >> 14) as i64 as u64;
                    self.write_reg(rd, imm);
                    self.pc = next_pc;
                }
            }

            (0b01, 0b100) => {
                // C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND, C.SUBW, C.ADDW
                let funct2 = (inst >> 10) & 0x3;
                let rd_p = ((inst >> 7) & 0x7) as usize + 8;
                
                match funct2 {
                    0b00 => {
                        // C.SRLI
                        let shamt = (((inst >> 12) & 0x1) << 5) | ((inst >> 2) & 0x1F);
                        let value = self.read_reg(rd_p) >> shamt;
                        self.write_reg(rd_p, value);
                        self.pc = next_pc;
                    }
                    0b01 => {
                        // C.SRAI
                        let shamt = (((inst >> 12) & 0x1) << 5) | ((inst >> 2) & 0x1F);
                        let value = ((self.read_reg(rd_p) as i64) >> shamt) as u64;
                        self.write_reg(rd_p, value);
                        self.pc = next_pc;
                    }
                    0b10 => {
                        // C.ANDI
                        let imm = (((inst >> 12) & 0x1) << 5) | ((inst >> 2) & 0x1F);
                        let imm = ((imm as i32) << 26 >> 26) as i64 as u64;
                        let value = self.read_reg(rd_p) & imm;
                        self.write_reg(rd_p, value);
                        self.pc = next_pc;
                    }
                    0b11 => {
                        // Arithmetic operations
                        let funct1 = (inst >> 12) & 0x1;
                        let funct2_low = (inst >> 5) & 0x3;
                        let rs2_p = ((inst >> 2) & 0x7) as usize + 8;
                        
                        match (funct1, funct2_low) {
                            (0, 0b00) => {
                                // C.SUB
                                let value = self.read_reg(rd_p).wrapping_sub(self.read_reg(rs2_p));
                                self.write_reg(rd_p, value);
                                self.pc = next_pc;
                            }
                            (0, 0b01) => {
                                // C.XOR
                                let value = self.read_reg(rd_p) ^ self.read_reg(rs2_p);
                                self.write_reg(rd_p, value);
                                self.pc = next_pc;
                            }
                            (0, 0b10) => {
                                // C.OR
                                let value = self.read_reg(rd_p) | self.read_reg(rs2_p);
                                self.write_reg(rd_p, value);
                                self.pc = next_pc;
                            }
                            (0, 0b11) => {
                                // C.AND
                                let value = self.read_reg(rd_p) & self.read_reg(rs2_p);
                                self.write_reg(rd_p, value);
                                self.pc = next_pc;
                            }
                            (1, 0b00) => {
                                // C.SUBW
                                let value = (self.read_reg(rd_p) as i32).wrapping_sub(self.read_reg(rs2_p) as i32);
                                self.write_reg(rd_p, value as i64 as u64);
                                self.pc = next_pc;
                            }
                            (1, 0b01) => {
                                // C.ADDW
                                let value = (self.read_reg(rd_p) as i32).wrapping_add(self.read_reg(rs2_p) as i32);
                                self.write_reg(rd_p, value as i64 as u64);
                                self.pc = next_pc;
                            }
                            _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                        }
                    }
                    _ => return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64)),
                }
            }

            (0b01, 0b101) => {
                // C.J
                let imm = (((inst >> 12) & 0x1) << 11)
                    | (((inst >> 11) & 0x1) << 4)
                    | (((inst >> 9) & 0x3) << 8)
                    | (((inst >> 8) & 0x1) << 10)
                    | (((inst >> 7) & 0x1) << 6)
                    | (((inst >> 6) & 0x1) << 7)
                    | (((inst >> 3) & 0x7) << 1)
                    | (((inst >> 2) & 0x1) << 5);
                let imm = ((imm as i32) << 20 >> 20) as i64 as u64;
                self.pc = self.pc.wrapping_add(imm);
            }

            (0b01, 0b110) => {
                // C.BEQZ
                let rs1_p = ((inst >> 7) & 0x7) as usize + 8;
                let imm = (((inst >> 12) & 0x1) << 8)
                    | (((inst >> 10) & 0x3) << 3)
                    | (((inst >> 5) & 0x3) << 6)
                    | (((inst >> 3) & 0x3) << 1)
                    | (((inst >> 2) & 0x1) << 5);
                let imm = ((imm as i32) << 23 >> 23) as i64 as u64;
                
                if self.read_reg(rs1_p) == 0 {
                    self.pc = self.pc.wrapping_add(imm);
                } else {
                    self.pc = next_pc;
                }
            }

            (0b01, 0b111) => {
                // C.BNEZ
                let rs1_p = ((inst >> 7) & 0x7) as usize + 8;
                let imm = (((inst >> 12) & 0x1) << 8)
                    | (((inst >> 10) & 0x3) << 3)
                    | (((inst >> 5) & 0x3) << 6)
                    | (((inst >> 3) & 0x3) << 1)
                    | (((inst >> 2) & 0x1) << 5);
                let imm = ((imm as i32) << 23 >> 23) as i64 as u64;
                
                if self.read_reg(rs1_p) != 0 {
                    self.pc = self.pc.wrapping_add(imm);
                } else {
                    self.pc = next_pc;
                }
            }

            // ============ C2 (opcode = 10) ============
            (0b10, 0b000) => {
                // C.SLLI
                let rd = ((inst >> 7) & 0x1F) as usize;
                let shamt = (((inst >> 12) & 0x1) << 5) | ((inst >> 2) & 0x1F);
                let value = self.read_reg(rd) << shamt;
                self.write_reg(rd, value);
                self.pc = next_pc;
            }

            (0b10, 0b001) => {
                // C.FLDSP - 从栈加载浮点双精度 (RV32DC/RV64DC)
                let rd = ((inst >> 7) & 0x1F) as usize;
                let imm = (((inst >> 12) & 0x1) << 5)
                    | (((inst >> 5) & 0x3) << 3)
                    | (((inst >> 2) & 0x7) << 6);
                let va = self.read_reg(2).wrapping_add(imm as u64);
                
                let value = self.load_va(va, 8)?;
                self.f_regs[rd] = value;
                
                // 设置 mstatus.FS = Dirty (3)
                let mstatus = self.csr.read(MSTATUS);
                let new_mstatus = (mstatus & !(0x3 << 13)) | (0x3 << 13);
                self.csr.write(MSTATUS, new_mstatus);
                
                self.pc = next_pc;
            }

            (0b10, 0b010) => {
                // C.LWSP
                let rd = ((inst >> 7) & 0x1F) as usize;
                if rd == 0 {
                    return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                }
                let imm = (((inst >> 12) & 0x1) << 5)
                    | (((inst >> 4) & 0x7) << 2)
                    | (((inst >> 2) & 0x3) << 6);
                let va = self.read_reg(2).wrapping_add(imm as u64);
                
                let value = self.load_va(va, 4)?;
                self.write_reg(rd, (value as u32) as i32 as i64 as u64);
                self.pc = next_pc;
            }

            (0b10, 0b011) => {
                // C.LDSP (RV64)
                let rd = ((inst >> 7) & 0x1F) as usize;
                if rd == 0 {
                    return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
                }
                let imm = (((inst >> 12) & 0x1) << 5)
                    | (((inst >> 5) & 0x3) << 3)
                    | (((inst >> 2) & 0x7) << 6);
                let va = self.read_reg(2).wrapping_add(imm as u64);
                
                let value = self.load_va(va, 8)?;
                self.write_reg(rd, value);
                self.pc = next_pc;
            }

            (0b10, 0b101) => {
                // C.FSDSP - 存储浮点双精度到栈 (RV32DC/RV64DC)
                let rs2 = ((inst >> 2) & 0x1F) as usize;
                let imm = (((inst >> 10) & 0x7) << 3) | (((inst >> 7) & 0x7) << 6);
                let va = self.read_reg(2).wrapping_add(imm as u64);
                
                let value = self.f_regs[rs2];
                self.store_va(va, value, 8)?;
                self.clear_reservation();
                self.pc = next_pc;
            }

            (0b10, 0b110) => {
                // C.SWSP
                let rs2 = ((inst >> 2) & 0x1F) as usize;
                let imm = (((inst >> 9) & 0xF) << 2) | (((inst >> 7) & 0x3) << 6);
                let va = self.read_reg(2).wrapping_add(imm as u64);
                let value = self.read_reg(rs2);
                
                self.store_va(va, value, 4)?;
                self.clear_reservation();
                self.pc = next_pc;
            }

            (0b10, 0b100) => {
                // C.JR / C.MV / C.JALR / C.ADD
                let rs1 = ((inst >> 7) & 0x1F) as usize;
                let rs2 = ((inst >> 2) & 0x1F) as usize;
                
                if ((inst >> 12) & 0x1) == 0 {
                    if rs2 == 0 {
                        // C.JR
                        self.pc = self.read_reg(rs1);
                    } else {
                        // C.MV
                        self.write_reg(rs1, self.read_reg(rs2));
                        self.pc = next_pc;
                    }
                } else {
                    if rs2 == 0 {
                        if rs1 == 0 {
                            // C.EBREAK - tval 应该是触发 EBREAK 的地址（PC）
                            return Err((Trap::Exception(Exception::Breakpoint), self.pc));
                        } else {
                            // C.JALR
                            self.write_reg(1, next_pc);
                            self.pc = self.read_reg(rs1);
                        }
                    } else {
                        // C.ADD
                        let value = self.read_reg(rs1).wrapping_add(self.read_reg(rs2));
                        self.write_reg(rs1, value);
                        self.pc = next_pc;
                    }
                }
            }

            (0b10, 0b111) => {
                // C.SDSP (RV64)
                let rs2 = ((inst >> 2) & 0x1F) as usize;
                let imm = (((inst >> 10) & 0x7) << 3) | (((inst >> 7) & 0x7) << 6);
                let va = self.read_reg(2).wrapping_add(imm as u64);
                let value = self.read_reg(rs2);
                
                self.store_va(va, value, 8)?;
                self.clear_reservation();
                self.pc = next_pc;
            }

            _ => {
                // 未实现的压缩指令
                eprintln!("ERROR: Unimplemented compressed instruction: 0x{:04x}, opcode=0b{:02b}, funct3=0b{:03b}", 
                          inst, opcode, funct3);
                return Err((Trap::Exception(Exception::IllegalInstruction), inst as u64));
            }
        }

        Ok(())
    }

    /// 检查并处理中断（严格遵循 RISC-V Spec v1.12）
    /// 
    /// # 中断检查逻辑：
    /// 1. 读取 MSTATUS.MIE（全局中断使能）
    /// 2. 查询硬件设备（PLIC、CLINT）更新 MIP
    /// 3. 计算 pending = MIE & MIP（同时使能且挂起的中断）
    /// 4. 根据 MIDELEG 判断是否委托给 S-Mode
    /// 5. 根据优先级和当前特权级触发中断
    /// 
    /// # RISC-V 中断优先级（同一特权级内）：
    /// - MEI (External) > MSI (Software) > MTI (Timer)
    /// - SEI (External) > SSI (Software) > STI (Timer)
    fn check_interrupt(&mut self) -> Result<(), Trap> {
        // ============ 步骤0: 同步设备中断状态到 PLIC ============
        // 模拟电平触发：真实硬件中 UART 中断线持续驱动 PLIC，
        // 但我们的模拟器只在 UART 寄存器访问时更新 PLIC。
        // 所以在每次中断检查前，必须主动同步一次。
        self.bus.update_uart_irq_public();
        
        // ============ 步骤1: 检查全局中断使能 ============
        let mstatus = self.csr.read(MSTATUS);
        let mie_enabled = (mstatus >> 3) & 0x1;  // MSTATUS.MIE (bit 3)
        let sie_enabled = (mstatus >> 1) & 0x1;  // MSTATUS.SIE (bit 1)
        
        // ============ 步骤2: 更新 MIP（查询硬件状态）============
        let mut mip = self.csr.read(MIP);
        
        // 2.1 外部中断 (MEIP, bit 11)：从 PLIC 查询（Context 0 = M-Mode）
        let has_external = self.bus.has_external_interrupt(0);
        if has_external {
            mip |= 1 << 11;  // 设置 MEIP
        } else {
            mip &= !(1 << 11);  // 清除 MEIP
        }
        
        // 2.2 M-Mode 定时器中断 (MTIP, bit 7)：从 CLINT 查询 (mtime >= mtimecmp)
        let has_timer = self.bus.has_timer_interrupt();
        if has_timer {
            mip |= 1 << 7;  // 设置 MTIP
        } else {
            mip &= !(1 << 7);  // 清除 MTIP
        }
        
        // 2.3 软件中断 (MSIP, bit 3)：从 CLINT 查询（目前未实现，保留逻辑）
        let has_software = self.bus.has_software_interrupt(0);
        if has_software {
            mip |= 1 << 3;  // 设置 MSIP
        } else {
            mip &= !(1 << 3);  // 清除 MSIP
        }
        
        // 2.4 S-Mode 外部中断 (SEIP, bit 9)：从 PLIC 查询（Context 1 = S-Mode）
        let has_s_external = self.bus.has_external_interrupt(1);
        if has_s_external {
            mip |= 1 << 9;  // 设置 SEIP
        } else {
            mip &= !(1 << 9);  // 清除 SEIP
        }

        // 2.5 S-Mode 定时器中断 (STIP, bit 5)：Sstc 扩展
        // 当 mtime >= stimecmp 时，硬件自动设置 STIP
        if self.bus.has_s_timer_interrupt() {
            mip |= 1 << 5;  // 设置 STIP
        } else {
            mip &= !(1 << 5);  // 清除 STIP
        }

        // 写回 MIP（注意：只更新硬件控制的位）
        self.csr.write(MIP, mip);
        
        // ============ 步骤3: 计算待处理中断 ============
        let mie = self.csr.read(MIE);
        let pending = mip & mie;  // 同时使能且挂起的中断
        
        if pending == 0 {
            return Ok(());  // 没有待处理中断
        }
        
        // ============ 步骤4: 读取中断委托寄存器 ============
        let mideleg = self.csr.read(MIDELEG);
        
        // ============ 步骤5: 根据当前特权级和优先级判断中断 ============
        
        // 5.1 M-Mode 中断（优先级：MEI > MSI > MTI）
        // 注意：M-Mode 中断在所有特权级都可以触发（如果 MSTATUS.MIE=1）
        if self.mode == Mode::Machine && mie_enabled == 1 {
            // Machine External Interrupt (bit 11)
            if (pending & (1 << 11)) != 0 {
                return Err(Trap::Interrupt(Interrupt::MachineExternalInterrupt));
            }
            // Machine Software Interrupt (bit 3)
            if (pending & (1 << 3)) != 0 {
                return Err(Trap::Interrupt(Interrupt::MachineSoftwareInterrupt));
            }
            // Machine Timer Interrupt (bit 7)
            if (pending & (1 << 7)) != 0 {
                return Err(Trap::Interrupt(Interrupt::MachineTimerInterrupt));
            }
        }
        
        // 5.2 S-Mode 中断（优先级：SEI > SSI > STI）
        // 只有当中断被委托且 SSTATUS.SIE=1 时才触发
        if self.mode == Mode::Supervisor && sie_enabled == 1 {
            // Supervisor External Interrupt (bit 9)
            if (pending & (1 << 9)) != 0 && (mideleg & (1 << 9)) != 0 {
                return Err(Trap::Interrupt(Interrupt::SupervisorExternalInterrupt));
            }
            // Supervisor Software Interrupt (bit 1)
            if (pending & (1 << 1)) != 0 && (mideleg & (1 << 1)) != 0 {
                return Err(Trap::Interrupt(Interrupt::SupervisorSoftwareInterrupt));
            }
            // Supervisor Timer Interrupt (bit 5)
            if (pending & (1 << 5)) != 0 && (mideleg & (1 << 5)) != 0 {
                return Err(Trap::Interrupt(Interrupt::SupervisorTimerInterrupt));
            }
        }
        
        // 5.3 U-Mode 中断（通常不使用，暂不实现）
        
        // ============ 步骤6: 特殊情况：低特权级代码运行时，高特权级中断仍可抢占 ============
        // 如果当前在 S-Mode 或 U-Mode，且 M-Mode 中断挂起，立即触发
        if self.mode != Mode::Machine {
            // Machine External Interrupt (bit 11)
            if (pending & (1 << 11)) != 0 {
                return Err(Trap::Interrupt(Interrupt::MachineExternalInterrupt));
            }
            // Machine Software Interrupt (bit 3)
            if (pending & (1 << 3)) != 0 {
                return Err(Trap::Interrupt(Interrupt::MachineSoftwareInterrupt));
            }
            // Machine Timer Interrupt (bit 7)
            if (pending & (1 << 7)) != 0 {
                return Err(Trap::Interrupt(Interrupt::MachineTimerInterrupt));
            }
        }
        
        // 如果当前在 U-Mode，且 S-Mode 中断挂起且已委托，立即触发
        // 注意：U-Mode 代码可以被 S-Mode 中断打断，不需要检查 SIE！
        // 只有在 S-Mode 运行时，SIE 才控制是否处理 S-Mode 中断
        if self.mode == Mode::User {
            // Supervisor External Interrupt (bit 9)
            if (pending & (1 << 9)) != 0 && (mideleg & (1 << 9)) != 0 {
                return Err(Trap::Interrupt(Interrupt::SupervisorExternalInterrupt));
            }
            // Supervisor Software Interrupt (bit 1)
            if (pending & (1 << 1)) != 0 && (mideleg & (1 << 1)) != 0 {
                return Err(Trap::Interrupt(Interrupt::SupervisorSoftwareInterrupt));
            }
            // Supervisor Timer Interrupt (bit 5)
            if (pending & (1 << 5)) != 0 && (mideleg & (1 << 5)) != 0 {
                return Err(Trap::Interrupt(Interrupt::SupervisorTimerInterrupt));
            }
        }
        
        Ok(())
    }

    /// 更新定时器（由主循环定期调用）
    pub fn update_timer(&mut self) {
        self.update_timer_by(1);
    }

    /// 更新定时器（步进可配置）
    pub fn update_timer_by(&mut self, delta: u64) {
        // 仅推进 CLINT mtime，MIP.MTIP 在 check_interrupt 中更新
        self.bus.tick_timer_by(delta);
    }
}

/// 获取寄存器的ABI名称
pub fn reg_name(index: usize) -> &'static str {
    match index {
        0 => "zero",
        1 => "ra",
        2 => "sp",
        3 => "gp",
        4 => "tp",
        5 => "t0",
        6 => "t1",
        7 => "t2",
        8 => "s0",
        9 => "s1",
        10 => "a0",
        11 => "a1",
        12 => "a2",
        13 => "a3",
        14 => "a4",
        15 => "a5",
        16 => "a6",
        17 => "a7",
        18 => "s2",
        19 => "s3",
        20 => "s4",
        21 => "s5",
        22 => "s6",
        23 => "s7",
        24 => "s8",
        25 => "s9",
        26 => "s10",
        27 => "s11",
        28 => "t3",
        29 => "t4",
        30 => "t5",
        31 => "t6",
        _ => "??",
    }
}
