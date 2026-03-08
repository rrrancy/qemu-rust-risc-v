// trap.rs - 异常和中断处理

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exception {
    // 指令地址未对齐
    InstructionAddressMisaligned = 0,
    // 指令访问错误
    InstructionAccessFault = 1,
    // 非法指令
    IllegalInstruction = 2,
    // 断点
    Breakpoint = 3,
    // 加载地址未对齐
    LoadAddressMisaligned = 4,
    // 加载访问错误
    LoadAccessFault = 5,
    // 存储/AMO 地址未对齐
    StoreAMOAddressMisaligned = 6,
    // 存储/AMO 访问错误
    StoreAMOAccessFault = 7,
    // 环境调用 (U-mode)
    EnvironmentCallFromUMode = 8,
    // 环境调用 (S-mode)
    EnvironmentCallFromSMode = 9,
    // 环境调用 (M-mode)
    EnvironmentCallFromMMode = 11,
    // 指令页错误
    InstructionPageFault = 12,
    // 加载页错误
    LoadPageFault = 13,
    // 存储/AMO 页错误
    StoreAMOPageFault = 15,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Interrupt {
    // 用户软件中断
    UserSoftwareInterrupt = 0,
    // 监督者软件中断
    SupervisorSoftwareInterrupt = 1,
    // 机器软件中断
    MachineSoftwareInterrupt = 3,
    // 用户定时器中断
    UserTimerInterrupt = 4,
    // 监督者定时器中断
    SupervisorTimerInterrupt = 5,
    // 机器定时器中断
    MachineTimerInterrupt = 7,
    // 用户外部中断
    UserExternalInterrupt = 8,
    // 监督者外部中断
    SupervisorExternalInterrupt = 9,
    // 机器外部中断
    MachineExternalInterrupt = 11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trap {
    Exception(Exception),
    Interrupt(Interrupt),
}

impl Trap {
    /// 获取异常/中断编码（用于写入 mcause/scause）
    pub fn code(&self) -> u64 {
        match self {
            Trap::Exception(e) => *e as u64,
            Trap::Interrupt(i) => (1u64 << 63) | (*i as u64),
        }
    }

    /// 判断是否为中断
    pub fn is_interrupt(&self) -> bool {
        matches!(self, Trap::Interrupt(_))
    }

    /// 判断是否为异常
    pub fn is_exception(&self) -> bool {
        matches!(self, Trap::Exception(_))
    }
}
