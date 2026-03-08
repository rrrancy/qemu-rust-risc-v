// csr.rs - Control and Status Registers (CSR)
// 实现 RISC-V 64-bit 特权架构的 CSR 寄存器

use std::collections::HashMap;

// ============ CSR 地址常量 ============
// Machine-level CSRs
pub const MSTATUS: u16 = 0x300;
pub const MISA: u16 = 0x301;
pub const MEDELEG: u16 = 0x302;
pub const MIDELEG: u16 = 0x303;
pub const MIE: u16 = 0x304;
pub const MTVEC: u16 = 0x305;
pub const MCOUNTEREN: u16 = 0x306;
pub const MENVCFG: u16 = 0x30A;
pub const MCOUNTINHIBIT: u16 = 0x320;
// PMU Event Selectors: mhpmevent3-18 (0x323-0x332)
// 对应 mhpmcounter3-18 (0xB03-0xB12)
pub const MHPMEVENT3_LO: u16 = 0x323;
pub const MHPMEVENT18_HI: u16 = 0x332;

pub const MSCRATCH: u16 = 0x340;
pub const MEPC: u16 = 0x341;
pub const MCAUSE: u16 = 0x342;
pub const MTVAL: u16 = 0x343;
pub const MIP: u16 = 0x344;
pub const MTINST: u16 = 0x34A;
pub const MTVAL2: u16 = 0x34B;

// PMP Configuration Registers
pub const PMPCFG0: u16 = 0x3A0;
pub const PMPCFG2: u16 = 0x3A2;

pub const MVENDORID: u16 = 0xF11;
pub const MARCHID: u16 = 0xF12;
pub const MIMPID: u16 = 0xF13;
pub const MHARTID: u16 = 0xF14;

// Supervisor-level CSRs
pub const SSTATUS: u16 = 0x100;
pub const SEDELEG: u16 = 0x102;
pub const SIDELEG: u16 = 0x103;
pub const SIE: u16 = 0x104;
pub const STVEC: u16 = 0x105;
pub const SCOUNTEREN: u16 = 0x106;
pub const SENVCFG: u16 = 0x10A;  // S-mode 环境配置 (从 MENVCFG 派生)

pub const SSCRATCH: u16 = 0x140;
pub const SEPC: u16 = 0x141;
pub const SCAUSE: u16 = 0x142;
pub const STVAL: u16 = 0x143;
pub const SIP: u16 = 0x144;
pub const STIMECMP: u16 = 0x14D;

pub const SATP: u16 = 0x180;

// Hypervisor CSRs (RV64 H 扩展)
pub const HSTATUS: u16 = 0x600;
pub const HEDELEG: u16 = 0x602;
pub const HIDELEG: u16 = 0x603;
pub const HCOUNTEREN: u16 = 0x606;
pub const HTVAL: u16 = 0x643;
pub const HTINST: u16 = 0x64A;

pub const VSSTATUS: u16 = 0x200;
pub const VSIE: u16 = 0x204;
pub const VSTVEC: u16 = 0x205;
pub const VSSCRATCH: u16 = 0x240;
pub const VSEPC: u16 = 0x241;
pub const VSCAUSE: u16 = 0x242;
pub const VSTVAL: u16 = 0x243;
pub const VSIP: u16 = 0x244;
pub const VSATP: u16 = 0x280;

// Floating-Point CSRs
pub const FFLAGS: u16 = 0x001;  // 浮点异常标志 (FCSR[4:0] 的别名)
pub const FRM: u16 = 0x002;     // 浮点舍入模式 (FCSR[7:5] 的别名)
pub const FCSR: u16 = 0x003;    // 浮点控制状态寄存器

// Debug CSRs (Trigger Module)
pub const TSELECT: u16 = 0x7A0;
pub const TDATA1: u16 = 0x7A1;
pub const TDATA2: u16 = 0x7A2;
pub const TDATA3: u16 = 0x7A3;
pub const TINFO: u16 = 0x7A4;

// User-level CSRs (Counters)
pub const CYCLE: u16 = 0xC00;
pub const TIME: u16 = 0xC01;
pub const INSTRET: u16 = 0xC02;

// PMU event counters (只支持 mhpmevent3-7，对齐 QEMU)
pub const MHPMEVENT3: u16 = 0xB03;
pub const MHPMEVENT7: u16 = 0xB07;

pub struct Csr {
    regs: HashMap<u16, u64>,
    // Debug Trigger 模块配置
    num_triggers: u64,  // 支持的触发器数量 (对齐 QEMU virt，只支持 2 个: 索引 0-1)
}

impl Csr {
    pub fn new() -> Self {
        let mut csr = Self {
            regs: HashMap::new(),
            num_triggers: 2,  // 对齐 QEMU virt：只支持 2 个触发器 (索引 0-1)
        };

        // 根据 qemu_golden_trace.log 初始化 CSR (必须与 QEMU 一致！)
        csr.write(MSTATUS, 0x0000_000a_0000_0000);  // MPP=0, MPIE=1, MIE=0
        // MISA: RV64 + ISA 扩展 (对齐 QEMU virt)
        csr.write(MISA, 0x8000_0000_0014_11ad);
        // MENVCFG: Machine Environment Configuration
        // bit 63 (STCE) = 1: 允许 S-mode 访问 stimecmp CSR (Sstc 扩展)
        // bit 61 (CBZE) = 1: 允许使用 cbo.zero 指令
        // ⚠️ 关键：STCE 必须为 1，否则 Linux 内核不会使用 Sstc 扩展！
        csr.write(MENVCFG, 0xA000_0000_0000_0000);  // STCE(bit63)=1, CBZE(bit61)=1
        csr.write(MCOUNTINHIBIT, 0);  // 性能计数器控制（初始不暂停任何计数器）
        
        // 初始化 PMU Event Selectors (mhpmevent3-18: 0x323-0x332)
        for addr in 0x323..=0x332 {
            csr.write(addr, 0);
        }
        
        csr.write(MEDELEG, 0x0000_0000_00f4_b509);  // 委托异常给 S-mode
        csr.write(MIDELEG, 0x0000_0000_0000_1666);  // 委托中断给 S-mode (包含 SSIP/STIP/SEIP)
        csr.write(MIE, 0);
        csr.write(MTVEC, 0);
        csr.write(MEPC, 0);
        csr.write(MCAUSE, 0);
        csr.write(MTVAL, 0);
        csr.write(MIP, 0x0000_0000_0000_0080);      // 定时器中断待处理
        csr.write(MSCRATCH, 0);

        csr.write(SSTATUS, 0);
        csr.write(SIE, 0);
        csr.write(STVEC, 0);
        csr.write(SEPC, 0);
        csr.write(SCAUSE, 0);
        csr.write(STVAL, 0);
        csr.write(SIP, 0);
        csr.write(SSCRATCH, 0);
        csr.write(STIMECMP, u64::MAX);  // Supervisor Time Compare (Sstc 扩展)，初始禁用
        csr.write(SATP, 0);

        csr.write(HSTATUS, 0x0000_0002_0000_0000);   // H 扩展状态
        csr.write(HEDELEG, 0);
        csr.write(HIDELEG, 0);
        csr.write(HTVAL, 0);
        csr.write(HTINST, 0);

        csr.write(VSSTATUS, 0x0000_000a_0000_0000);  // 虚拟化 S-mode 状态
        csr.write(VSIE, 0);
        csr.write(VSTVEC, 0);
        csr.write(VSEPC, 0);
        csr.write(VSCAUSE, 0);
        csr.write(VSTVAL, 0);
        csr.write(VSIP, 0);
        csr.write(VSSCRATCH, 0);
        csr.write(VSATP, 0);

        // Debug CSRs (Trigger Module)
        csr.write(TSELECT, 0);  // Trigger Select
        csr.write(TDATA1, 0);   // Trigger Data 1
        csr.write(TDATA2, 0);   // Trigger Data 2
        csr.write(TDATA3, 0);   // Trigger Data 3
        // TINFO 是只读寄存器，在 read() 中特殊处理，总是返回 0x44

        // Floating-Point CSRs
        csr.write(FCSR, 0);  // Floating-Point Control and Status Register

        // mhartid 固定为 0 (单核)
        csr.write(MHARTID, 0);

        csr
    }

    /// 获取 CSR 所需的最低特权级
    /// CSR 地址格式: bits [11:10] = 特权级要求 (0=U, 1=S, 2=H, 3=M)
    /// 返回值: 0=User, 1=Supervisor, 2=Hypervisor, 3=Machine
    #[inline]
    pub fn get_csr_privilege(addr: u16) -> u8 {
        ((addr >> 8) & 0x3) as u8
    }
    
    /// 检查 CSR 是否为只读
    /// CSR 地址格式: bits [11:10] = 读写属性 (11=只读)
    #[inline]
    pub fn is_csr_readonly(addr: u16) -> bool {
        ((addr >> 10) & 0x3) == 0x3
    }
    
    /// 检查当前特权级是否有权限访问指定的 CSR
    /// mode: 当前 CPU 特权模式 (0=U, 1=S, 2=H, 3=M)
    /// is_write: 是否为写操作
    /// 返回 true 表示有权限访问
    pub fn check_csr_privilege(addr: u16, mode: u8, is_write: bool) -> bool {
        let required_priv = Self::get_csr_privilege(addr);
        
        // 1. 检查特权级是否足够
        if mode < required_priv {
            return false;
        }
        
        // 2. 检查是否尝试写入只读 CSR
        if is_write && Self::is_csr_readonly(addr) {
            return false;
        }
        
        true
    }

    /// 检查 CSR 地址是否合法
    /// 根据 QEMU 行为，只允许访问特定的 CSR
    pub fn is_valid_csr(addr: u16) -> bool {
        match addr {
            // Machine-level CSRs
            MSTATUS | MISA | MEDELEG | MIDELEG | MIE | MTVEC | MCOUNTEREN => true,
            MENVCFG | MCOUNTINHIBIT => true,
            MSCRATCH | MEPC | MCAUSE | MTVAL | MIP | MTINST | MTVAL2 => true,
            MVENDORID | MARCHID | MIMPID | MHARTID => true,
            
            // PMU Event Selectors: mhpmevent3-18 (0x323-0x332)
            // 必须与 mhpmcounter3-18 (0xB03-0xB12) 数量一致
            0x323..=0x332 => true,
            
            // PMP Configuration and Address Registers
            // PMPCFG0-PMPCFG3 (0x3A0-0x3A3) for RV64
            0x3A0..=0x3A3 => true,
            // PMPADDR0-PMPADDR63 (0x3B0-0x3EF)
            0x3B0..=0x3EF => true,
            
            // Supervisor-level CSRs
            SSTATUS | SEDELEG | SIDELEG | SIE | STVEC | SCOUNTEREN | SENVCFG => true,
            SSCRATCH | SEPC | SCAUSE | STVAL | SIP | STIMECMP => true,
            SATP => true,
            
            // Hypervisor CSRs
            HSTATUS | HEDELEG | HIDELEG | HCOUNTEREN | HTVAL | HTINST => true,
            VSSTATUS | VSIE | VSTVEC | VSSCRATCH | VSEPC | VSCAUSE | VSTVAL | VSIP | VSATP => true,
            
            // Debug CSRs (Trigger Module)
            TSELECT | TDATA1 | TDATA2 | TDATA3 | TINFO => true,
            
            // Floating-Point CSRs
            FFLAGS | FRM | FCSR => true,
            
            // User-level CSRs
            CYCLE | TIME | INSTRET => true,
            
            // PMU Counters: 只允许 0xB03-0xB12 (mhpmcounter3-18)
            // QEMU 的 virt 平台支持到 counter18，拒绝 counter19 (0xB13) 及以上
            0xB03..=0xB12 => true,
            
            // 注意：0x3c0 已被 PMPADDR 范围 (0x3B0..=0x3EF) 覆盖，无需单独列出
            
            // 其他所有 CSR 都不合法
            _ => false,
        }
    }

    /// 读取 CSR 寄存器
    pub fn read(&self, addr: u16) -> u64 {
        match addr {
            // 0x3c0 是未定义CSR，QEMU返回0
            0x3c0 => 0,
            // tinfo (0x7A4) 是只读寄存器，固定返回 0x44
            TINFO => 0x44,
            
            // ========== RV64 架构 XLEN 字段强制修复 ==========
            
            // MISA: 需要强制 MXL (bits 63:62) 为 2 (64-bit)
            MISA => {
                let raw = *self.regs.get(&addr).unwrap_or(&0);
                // 强制 MXL (bits 63:62) 为 2 (64-bit)
                // 清除 bits 63:62，然后设置为 0b10
                let mut value = raw & !(0x3_u64 << 62);  // 清除 bits 63:62
                value |= 2_u64 << 62;  // 设置 MXL = 2 (64-bit)
                value
            },
            
            // MSTATUS: 需要动态计算 SD 位 (bit 63) 并强制 SXL/UXL
            MSTATUS => {
                let raw = *self.regs.get(&addr).unwrap_or(&0);
                let fs = (raw >> 13) & 0x3;  // FS 字段 (bits 14:13)
                let vs = (raw >> 9) & 0x3;   // VS 字段 (bits 10:9)
                // 强制 SXL/UXL 为 64-bit (bits 35:34 = SXL = 2, bits 33:32 = UXL = 2)
                // 清除 bits 32-35，设置为 0b1010
                let mut value = raw & !(0xF_u64 << 32);
                value |= 0xA_u64 << 32;  // SXL=2, UXL=2
                // SD (bit 63) = 1 当且仅当 FS==3 或 VS==3 (Dirty 状态)
                let sd = if fs == 3 || vs == 3 { 1u64 << 63 } else { 0 };
                // 清除原有的 SD 位，然后设置新的 SD 位
                (value & !(1u64 << 63)) | sd
            },
            
            // ⚠️ 关键修复：SSTATUS 是 MSTATUS 的视图（不是独立寄存器！）
            // SSTATUS 只能看到 MSTATUS 中 S-Mode 可访问的位
            // 可见位掩码：SD, UXL, MXR, SUM, XS, FS, VS, SPP, UBE, SPIE, SIE
            SSTATUS => {
                // 从 MSTATUS 读取并应用掩码
                const SSTATUS_MASK: u64 = 0x8000_0003_000D_E762;
                // SD(63) | UXL(33:32) | MXR(19) | SUM(18) | XS(16:15) | FS(14:13) | 
                // VS(10:9) | SPP(8) | UBE(6) | SPIE(5) | SIE(1)
                let mstatus = self.read(MSTATUS);
                mstatus & SSTATUS_MASK
            },
            
            // ⚠️ 关键修复：SIE 是 MIE 的视图
            // S-Mode 只能看到 SSIP(1), STIP(5), SEIP(9) 等位
            SIE => {
                const SIE_MASK: u64 = 0x222;  // bits 1, 5, 9
                let mie = *self.regs.get(&MIE).unwrap_or(&0);
                mie & SIE_MASK
            },
            
            // ⚠️ 关键修复：SIP 是 MIP 的视图
            SIP => {
                const SIP_MASK: u64 = 0x222;  // bits 1, 5, 9
                let mip = *self.regs.get(&MIP).unwrap_or(&0);
                mip & SIP_MASK
            },
            
            // ⚠️ 关键修复：SENVCFG 从 MENVCFG 派生
            // S-mode 读取 SENVCFG 来检查 STCE 位（bit 63）是否可用
            SENVCFG => {
                // S-mode 看到的是 MENVCFG 的值
                *self.regs.get(&MENVCFG).unwrap_or(&0)
            },
            
            // HSTATUS: H 扩展状态寄存器
            // 需要强制 VSXL (bits 33:32) 为 2 (64-bit)
            HSTATUS => {
                let raw = *self.regs.get(&addr).unwrap_or(&0);
                // 强制 VSXL (bits 33:32) 为 2 (64-bit 模式)
                let mut value = raw & !(0x3_u64 << 32);  // 清除 bits 33:32
                value |= 2_u64 << 32;  // 设置 VSXL = 2 (64-bit)
                value
            },
            
            // VSSTATUS: VS-Mode 状态寄存器
            // 需要强制 UXL (bits 33:32) 为 2 (64-bit)
            VSSTATUS => {
                let raw = *self.regs.get(&addr).unwrap_or(&0);
                // 强制 UXL (bits 33:32) 为 2 (64-bit)
                let mut value = raw & !(0x3_u64 << 32);  // 清除 bits 33:32
                value |= 2_u64 << 32;  // 设置 UXL = 2 (64-bit)
                value
            },
            
            // FFLAGS (0x001): 浮点异常标志，是 FCSR[4:0] 的别名
            FFLAGS => {
                let fcsr = *self.regs.get(&FCSR).unwrap_or(&0);
                fcsr & 0x1F  // 返回 bits [4:0]
            },
            
            // FRM (0x002): 浮点舍入模式，是 FCSR[7:5] 的别名
            FRM => {
                let fcsr = *self.regs.get(&FCSR).unwrap_or(&0);
                (fcsr >> 5) & 0x7  // 返回 bits [7:5]
            },
            
            _ => *self.regs.get(&addr).unwrap_or(&0),
        }
    }

    /// 写入 CSR 寄存器
    pub fn write(&mut self, addr: u16, value: u64) {
        match addr {
            // 0x3c0 是未定义CSR，QEMU忽略写入
            0x3c0 => {},
            // tinfo (0x7A4) 是只读寄存器，忽略写入
            TINFO => {},
            // MSTATUS: SD 位 (bit 63) 是只读的，软件写入时应该被忽略
            MSTATUS => {
                let old_mstatus = *self.regs.get(&MSTATUS).unwrap_or(&0);
                // 清除 SD 位（bit 63），保持其他位
                let masked_value = value & !(1u64 << 63);
                self.regs.insert(addr, masked_value);
            },
            // ⚠️ 关键修复：SSTATUS 写入实际上修改 MSTATUS 的对应位
            SSTATUS => {
                const SSTATUS_MASK: u64 = 0x8000_0003_000D_E762;
                // 读取当前 MSTATUS，保留 S-Mode 不可见的位，更新可见位
                let old_mstatus = *self.regs.get(&MSTATUS).unwrap_or(&0);
                let new_mstatus = (old_mstatus & !SSTATUS_MASK) | (value & SSTATUS_MASK);
                // 清除 SD 位（只读）
                let masked_value = new_mstatus & !(1u64 << 63);
                self.regs.insert(MSTATUS, masked_value);
            },
            MIE => {
                self.regs.insert(addr, value);
            },
            // FFLAGS (0x001): 写入时更新 FCSR[4:0]
            FFLAGS => {
                let old_fcsr = *self.regs.get(&FCSR).unwrap_or(&0);
                let new_fcsr = (old_fcsr & !0x1F) | (value & 0x1F);
                self.regs.insert(FCSR, new_fcsr);
            },
            // FRM (0x002): 写入时更新 FCSR[7:5]
            FRM => {
                let old_fcsr = *self.regs.get(&FCSR).unwrap_or(&0);
                let new_fcsr = (old_fcsr & !0xE0) | ((value & 0x7) << 5);
                self.regs.insert(FCSR, new_fcsr);
            },
            // ⚠️ 关键修复：SIE 写入实际上修改 MIE 的对应位
            SIE => {
                const SIE_MASK: u64 = 0x222;  // bits 1, 5, 9
                let old_mie = *self.regs.get(&MIE).unwrap_or(&0);
                let new_mie = (old_mie & !SIE_MASK) | (value & SIE_MASK);
                self.regs.insert(MIE, new_mie);
            },
            // ⚠️ 关键修复：SIP 写入实际上修改 MIP 的对应位（仅限 SSIP）
            SIP => {
                // 注意：SIP 中只有 SSIP (bit 1) 是软件可写的
                const SIP_WRITABLE: u64 = 0x2;  // 只有 bit 1 (SSIP)
                let old_mip = *self.regs.get(&MIP).unwrap_or(&0);
                let new_mip = (old_mip & !SIP_WRITABLE) | (value & SIP_WRITABLE);
                self.regs.insert(MIP, new_mip);
            },
            // tselect (0x7A0): 触发器选择寄存器
            // 只允许写入 0 到 (num_triggers-1) 的值
            // 如果写入值超出范围，忽略写入并保持原值（对齐 QEMU 行为）
            TSELECT => {
                if value < self.num_triggers {
                    self.regs.insert(addr, value);
                }
                // 否则忽略写入，保持原值不变
            },
            // MIDELEG: 中断委托寄存器
            // 根据 RISC-V 规范，只有某些中断可以被委托给 S 模式
            // 可委托的中断包括：SSIP(bit1), STIP(bit5), SEIP(bit9) 等
            // ⚠️ 关键：QEMU virt 实现中，某些位是"强制委托"的，不能被清除
            // 这是为了确保虚拟化和 S 模式的正确功能
            MIDELEG => {
                // QEMU virt 的强制委托位：bit 2, 6, 10, 12 (0x1444)
                // 这些位一旦在初始化时设置，就不能被软件清除
                const MIDELEG_FORCE: u64 = 0x1444; // 强制保持为1的位
                const MIDELEG_WRITABLE: u64 = 0xFFFF; // 可写位（允许所有低16位）
                
                // 计算新值：软件写入的值 OR 强制位
                let masked_value = (value & MIDELEG_WRITABLE) | MIDELEG_FORCE;
                self.regs.insert(addr, masked_value);
            },
            // MIP: 中断待处理寄存器
            // 根据 RISC-V 规范，某些位是只读的（由硬件控制）
            // 软件只能写入某些位（如 SSIP, STIP, SEIP）
            MIP => {
                // MIP 的可写位掩码（对齐 QEMU 行为）
                // 通常 MTIP(bit7), MSIP(bit3), MEIP(bit11) 由硬件控制，软件不可写
                // SSIP(bit1), STIP(bit5), SEIP(bit9) 可以被软件写入（如果委托了）
                const MIP_WRITABLE: u64 = 0x0222; // bit 1, 5, 9 (SSIP, STIP, SEIP)
                let old = self.read(addr);
                // 保留只读位，只更新可写位
                let new_value = (old & !MIP_WRITABLE) | (value & MIP_WRITABLE);
                self.regs.insert(addr, new_value);
            },
            SATP => {
                // ⚠️ SATP mode 支持
                // 支持 Mode 0 (Bare), Mode 8 (Sv39), Mode 9 (Sv48), Mode 10 (Sv57)
                let mode = (value >> 60) & 0xF;
                let mode_name = match mode {
                    0 => "Bare",
                    8 => "Sv39",
                    9 => "Sv48",
                    10 => "Sv57",
                    _ => "Unknown",
                };
                
                // 检查是否是支持的模式 (现在支持 Sv57!)
                let is_supported = mode == 0 || mode == 8 || mode == 9 || mode == 10;
                
                if !is_supported {
                    // 不支持的模式：忽略写入，SATP 保持不变
                    return;
                }
                
                self.regs.insert(addr, value);
            },
            STVEC => {
                self.regs.insert(addr, value);
            },
            // 其他 Debug CSR (tdata1/2/3) 正常读写
            _ => { self.regs.insert(addr, value); }
        }
    }

    /// CSRRW: CSR Read and Write
    pub fn csrrw(&mut self, addr: u16, value: u64) -> u64 {
        let old = self.read(addr);
        self.write(addr, value);
        old
    }

    /// CSRRS: CSR Read and Set
    pub fn csrrs(&mut self, addr: u16, mask: u64) -> u64 {
        let old = self.read(addr);
        // ⚠️ 关键：mask=0时不写入（rs1=0的优化）
        if mask != 0 {
            self.write(addr, old | mask);
        }
        old
    }

    /// CSRRC: CSR Read and Clear
    pub fn csrrc(&mut self, addr: u16, mask: u64) -> u64 {
        let old = self.read(addr);
        // ⚠️ 关键：mask=0时不写入（rs1=0的优化）
        if mask != 0 {
            self.write(addr, old & !mask);
        }
        old
    }
    
}

impl Default for Csr {
    fn default() -> Self {
        Self::new()
    }
}
