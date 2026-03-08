// mmu.rs - RISC-V SV39 虚拟内存管理单元
// 实现三级页表遍历和地址转换

use crate::bus::Bus;
use crate::cpu::Mode;
use crate::trap::{Exception, Trap};

/// SV39 页表常量
const PAGE_SIZE: u64 = 4096;
const PAGE_SHIFT: u64 = 12;

/// SV39 虚拟地址布局 (39位有效地址)
/// [38:30] VPN[2] (9位)
/// [29:21] VPN[1] (9位)
/// [20:12] VPN[0] (9位)
/// [11:0]  页内偏移 (12位)
const VPN_MASK: u64 = 0x1FF; // 9位掩码

/// PTE (Page Table Entry) 标志位
const PTE_V: u64 = 1 << 0;  // Valid
const PTE_R: u64 = 1 << 1;  // Readable
const PTE_W: u64 = 1 << 2;  // Writable
const PTE_X: u64 = 1 << 3;  // Executable
const PTE_U: u64 = 1 << 4;  // User mode accessible
const PTE_G: u64 = 1 << 5;  // Global mapping
const PTE_A: u64 = 1 << 6;  // Accessed
const PTE_D: u64 = 1 << 7;  // Dirty

/// SATP 寄存器模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SatpMode {
    Bare = 0,  // 无分页 (物理地址直通)
    Sv39 = 8,  // 39位虚拟地址
    Sv48 = 9,  // 48位虚拟地址 (暂不支持)
    Sv57 = 10, // 57位虚拟地址 (5级页表)
}

/// 内存访问类型 (用于权限检查和异常分类)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessType {
    Instruction, // 取指令
    Load,        // 加载数据
    Store,       // 存储数据
}

/// MMU 实现
pub struct Mmu;

impl Mmu {
    /// 虚拟地址转物理地址 (公共接口)
    /// 
    /// 参数:
    /// - va: 虚拟地址
    /// - access_type: 访问类型 (Instruction/Load/Store)
    /// - mode: 当前特权模式
    /// - satp: SATP 寄存器值
    /// - mstatus: MSTATUS 寄存器值 (用于检查 MXR, SUM, MPRV, MPP 等)
    /// - bus: 总线接口 (用于读取页表)
    /// 
    /// 返回: 物理地址或页错误异常
    pub fn translate(
        va: u64,
        access_type: AccessType,
        mode: Mode,
        satp: u64,
        mstatus: u64,
        bus: &mut Bus,
    ) -> Result<u64, Trap> {
        // 1. 解析 SATP
        let satp_mode = (satp >> 60) & 0xF;
        
        // ⚠️ 关键修复：M-Mode 指令获取永远不使用地址翻译！
        // 根据 RISC-V 规范：
        // - M-Mode 的指令获取（Instruction Fetch）永远不经过 MMU
        // - MPRV 只影响 M-Mode 下的数据访问（Load/Store），不影响指令获取
        if mode == Mode::Machine && access_type == AccessType::Instruction {
            return Ok(va);  // M-Mode 指令获取直接使用物理地址
        }
        
        // 2. 确定有效特权级 (effective_mode)
        // MPRV (Bit 17): Modify PRiVilege
        // 当 MPRV=1 且当前在 M-Mode 且访问类型是数据访问时，
        // 使用 MPP (Bits 12:11) 作为有效特权级
        let effective_mode = if mode == Mode::Machine {
            // M-Mode 数据访问：检查 MPRV
            let mprv = (mstatus >> 17) & 0x1;
            if mprv == 1 {
                // 使用 MPP 作为有效特权级
                let mpp = (mstatus >> 11) & 0x3;
                match mpp {
                    0 => Mode::User,
                    1 => Mode::Supervisor,
                    2 => Mode::Hypervisor,
                    _ => Mode::Machine,
                }
            } else {
                Mode::Machine  // MPRV=0，M-Mode 数据访问不使用翻译
            }
        } else {
            mode
        };
        
        // 3. Machine Mode (effective) 不使用地址翻译
        if effective_mode == Mode::Machine {
            return Ok(va);
        }
        
        // 4. Bare Mode: 无分页，直接返回物理地址
        if satp_mode == SatpMode::Bare as u64 {
            return Ok(va);
        }
        
        // 5. 根据 SATP mode 选择页表遍历方式
        match satp_mode {
            8 => Self::translate_sv39(va, access_type, effective_mode, satp, mstatus, bus),
            9 => Self::translate_sv48(va, access_type, effective_mode, satp, mstatus, bus),
            10 => Self::translate_sv57(va, access_type, effective_mode, satp, mstatus, bus),
            _ => Err(Self::page_fault(access_type, va)),
        }
    }
    
    /// SV39 页表遍历实现
    fn translate_sv39(
        va: u64,
        access_type: AccessType,
        mode: Mode,
        satp: u64,
        mstatus: u64,
        bus: &mut Bus,
    ) -> Result<u64, Trap> {
        // 1. SV39 虚拟地址有效性检查 (bits 63:39 必须等于 bit 38)
        let va_sign = (va >> 38) & 0x1;
        let va_ext = (va >> 39) & 0x1FFFFFF;
        if (va_sign == 0 && va_ext != 0) || (va_sign == 1 && va_ext != 0x1FFFFFF) {
            return Err(Self::page_fault(access_type, va));
        }
        
        // 2. 提取 VPN[2:0] 和页内偏移
        let vpn = [
            (va >> 12) & VPN_MASK, // VPN[0]
            (va >> 21) & VPN_MASK, // VPN[1]
            (va >> 30) & VPN_MASK, // VPN[2]
        ];
        let page_offset = va & 0xFFF;
        
        // 3. 从 SATP 获取根页表物理地址
        let root_ppn = satp & 0x0FFF_FFFF_FFFF; // bits 43:0 (44位 PPN)
        let mut pte_addr = (root_ppn << PAGE_SHIFT) as u64;
        
        // 4. 三级页表遍历 (从 Level 2 到 Level 0)
        for level in (0..=2).rev() {
            // 计算 PTE 地址
            pte_addr += vpn[level] * 8;
            
            // 读取 PTE (64位)
            let pte = match bus.read64(pte_addr) {
                Ok(v) => v,
                Err(_) => {
                    return Err(Self::page_fault(access_type, va));
                }
            };
            
            // 检查 Valid 位
            if (pte & PTE_V) == 0 {
                return Err(Self::page_fault(access_type, va));
            }
            
            let pte_r = (pte & PTE_R) != 0;
            let pte_w = (pte & PTE_W) != 0;
            let pte_x = (pte & PTE_X) != 0;
            
            // ⚠️ 严格的 PTE 检查：R=0 且 W=1 是非法的（只写页不允许）
            if !pte_r && pte_w {
                return Err(Self::page_fault(access_type, va));
            }
            
            // 判断是否为叶子节点 (R/W/X 任一为 1)
            if pte_r || pte_w || pte_x {
                // 叶子节点：执行权限检查
                if let Err(e) = Self::check_permissions(pte, access_type, mode, mstatus) {
                    return Err(e);
                }
                
                // 计算物理地址
                let ppn = (pte >> 10) & 0x0FFF_FFFF_FFFF; // bits 53:10 (44位 PPN)
                
                // ⚠️ 超页对齐检查 (Critical!)
                let pa = if level == 2 {
                    // 1GB 超页：PPN[1:0] 必须为 0 (低 18 位)
                    if (ppn & 0x3FFFF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0x3FFFFFFF)
                } else if level == 1 {
                    // 2MB 超页：PPN[0] 必须为 0 (低 9 位)
                    if (ppn & 0x1FF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0x1FFFFF)
                } else {
                    // 4KB 常规页
                    (ppn << PAGE_SHIFT) | page_offset
                };
                
                // ⚠️ 关键修复：硬件 A/D 位更新
                // 检查并更新 Accessed (A) 和 Dirty (D) 位
                let pte_a = (pte & PTE_A) != 0;
                let pte_d = (pte & PTE_D) != 0;
                
                let mut updated_pte = pte;
                let mut needs_update = false;
                
                // 1. 任何访问都必须设置 A 位
                if !pte_a {
                    updated_pte |= PTE_A;
                    needs_update = true;
                }
                
                // 2. Store 访问必须设置 D 位
                if access_type == AccessType::Store && !pte_d {
                    updated_pte |= PTE_D;
                    needs_update = true;
                }
                
                // 3. 如果需要更新，将新 PTE 写回页表
                if needs_update {
                    // 尝试写回 PTE
                    if let Err(_) = bus.write64(pte_addr, updated_pte) {
                        // 写回失败（例如页表在 ROM 中），触发 Page Fault
                        return Err(Self::page_fault(access_type, va));
                    }
                }
                
                return Ok(pa);
            } else {
                // 非叶子节点：继续遍历下一级
                let ppn = (pte >> 10) & 0x0FFF_FFFF_FFFF;
                pte_addr = (ppn << PAGE_SHIFT) as u64;
            }
        }
        
        // 不应该到达这里 (遍历完成后必然找到叶子节点或失败)
        Err(Self::page_fault(access_type, va))
    }
    
    /// SV48 页表遍历实现 (4级页表)
    fn translate_sv48(
        va: u64,
        access_type: AccessType,
        mode: Mode,
        satp: u64,
        mstatus: u64,
        bus: &mut Bus,
    ) -> Result<u64, Trap> {
        // 1. SV48 虚拟地址有效性检查 (bits 63:48 必须等于 bit 47)
        let va_sign = (va >> 47) & 0x1;
        let va_ext = (va >> 48) & 0xFFFF;
        if (va_sign == 0 && va_ext != 0) || (va_sign == 1 && va_ext != 0xFFFF) {
            return Err(Self::page_fault(access_type, va));
        }
        
        // 2. 提取 VPN[3:0] 和页内偏移
        let vpn = [
            (va >> 12) & VPN_MASK, // VPN[0]
            (va >> 21) & VPN_MASK, // VPN[1]
            (va >> 30) & VPN_MASK, // VPN[2]
            (va >> 39) & VPN_MASK, // VPN[3]
        ];
        let page_offset = va & 0xFFF;
        
        // 3. 从 SATP 获取根页表物理地址
        let root_ppn = satp & 0x0FFF_FFFF_FFFF; // bits 43:0 (44位 PPN)
        let mut pte_addr = (root_ppn << PAGE_SHIFT) as u64;
        
        // 4. 四级页表遍历 (从 Level 3 到 Level 0)
        for level in (0..=3).rev() {
            // 计算 PTE 地址
            pte_addr += vpn[level] * 8;
            
            // 读取 PTE (64位)
            let pte = match bus.read64(pte_addr) {
                Ok(v) => v,
                Err(_) => {
                    return Err(Self::page_fault(access_type, va));
                }
            };
            
            // 检查 Valid 位
            if (pte & PTE_V) == 0 {
                return Err(Self::page_fault(access_type, va));
            }
            
            let pte_r = (pte & PTE_R) != 0;
            let pte_w = (pte & PTE_W) != 0;
            let pte_x = (pte & PTE_X) != 0;
            
            // ⚠️ 严格的 PTE 检查：R=0 且 W=1 是非法的（只写页不允许）
            if !pte_r && pte_w {
                return Err(Self::page_fault(access_type, va));
            }
            
            // 判断是否为叶子节点 (R/W/X 任一为 1)
            if pte_r || pte_w || pte_x {
                // 叶子节点：执行权限检查
                if let Err(e) = Self::check_permissions(pte, access_type, mode, mstatus) {
                    return Err(e);
                }
                
                // 计算物理地址
                let ppn = (pte >> 10) & 0x0FFF_FFFF_FFFF; // bits 53:10 (44位 PPN)
                
                // ⚠️ Sv48 超页对齐检查
                let pa = if level == 3 {
                    // 512GB 超页：PPN[2:0] 必须为 0 (低 27 位)
                    if (ppn & 0x7FFFFFF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0x7FFFFFFFFF) // 低 39 位
                } else if level == 2 {
                    // 1GB 超页：PPN[1:0] 必须为 0 (低 18 位)
                    if (ppn & 0x3FFFF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0x3FFFFFFF) // 低 30 位
                } else if level == 1 {
                    // 2MB 超页：PPN[0] 必须为 0 (低 9 位)
                    if (ppn & 0x1FF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0x1FFFFF) // 低 21 位
                } else {
                    // 4KB 常规页
                    (ppn << PAGE_SHIFT) | page_offset
                };
                
                // ⚠️ 硬件 A/D 位更新
                let pte_a = (pte & PTE_A) != 0;
                let pte_d = (pte & PTE_D) != 0;
                
                let mut updated_pte = pte;
                let mut needs_update = false;
                
                if !pte_a {
                    updated_pte |= PTE_A;
                    needs_update = true;
                }
                
                if access_type == AccessType::Store && !pte_d {
                    updated_pte |= PTE_D;
                    needs_update = true;
                }
                
                if needs_update {
                    if let Err(_) = bus.write64(pte_addr, updated_pte) {
                        return Err(Self::page_fault(access_type, va));
                    }
                }
                
                return Ok(pa);
            } else {
                // 非叶子节点：继续遍历下一级
                let ppn = (pte >> 10) & 0x0FFF_FFFF_FFFF;
                pte_addr = (ppn << PAGE_SHIFT) as u64;
            }
        }
        
        // 不应该到达这里
        Err(Self::page_fault(access_type, va))
    }
    
    /// SV57 页表遍历实现 (5级页表)
    fn translate_sv57(
        va: u64,
        access_type: AccessType,
        mode: Mode,
        satp: u64,
        mstatus: u64,
        bus: &mut Bus,
    ) -> Result<u64, Trap> {
        // 1. SV57 虚拟地址有效性检查 (bits 63:57 必须等于 bit 56)
        let va_sign = (va >> 56) & 0x1;
        let va_ext = (va >> 57) & 0x7F; // 高7位 (bits 63:57)
        if (va_sign == 0 && va_ext != 0) || (va_sign == 1 && va_ext != 0x7F) {
            return Err(Self::page_fault(access_type, va));
        }
        
        // 2. 提取 VPN[4:0] 和页内偏移
        let vpn = [
            (va >> 12) & VPN_MASK, // VPN[0]
            (va >> 21) & VPN_MASK, // VPN[1]
            (va >> 30) & VPN_MASK, // VPN[2]
            (va >> 39) & VPN_MASK, // VPN[3]
            (va >> 48) & VPN_MASK, // VPN[4]
        ];
        let page_offset = va & 0xFFF;
        
        // 3. 从 SATP 获取根页表物理地址
        let root_ppn = satp & 0x0FFF_FFFF_FFFF; // bits 43:0 (44位 PPN)
        let mut pte_addr = (root_ppn << PAGE_SHIFT) as u64;
        
        // 4. 五级页表遍历 (从 Level 4 到 Level 0)
        for level in (0..=4).rev() {
            // 计算 PTE 地址
            pte_addr += vpn[level] * 8;
            
            // 读取 PTE (64位)
            let pte = match bus.read64(pte_addr) {
                Ok(v) => v,
                Err(_) => {
                    return Err(Self::page_fault(access_type, va));
                }
            };
            
            // 检查 Valid 位
            if (pte & PTE_V) == 0 {
                return Err(Self::page_fault(access_type, va));
            }
            
            let pte_r = (pte & PTE_R) != 0;
            let pte_w = (pte & PTE_W) != 0;
            let pte_x = (pte & PTE_X) != 0;
            
            // ⚠️ 严格的 PTE 检查：R=0 且 W=1 是非法的（只写页不允许）
            if !pte_r && pte_w {
                return Err(Self::page_fault(access_type, va));
            }
            
            // 判断是否为叶子节点 (R/W/X 任一为 1)
            if pte_r || pte_w || pte_x {
                // 叶子节点：执行权限检查
                if let Err(e) = Self::check_permissions(pte, access_type, mode, mstatus) {
                    return Err(e);
                }
                
                // 计算物理地址
                let ppn = (pte >> 10) & 0x0FFF_FFFF_FFFF; // bits 53:10 (44位 PPN)
                
                // ⚠️ Sv57 超页对齐检查
                let pa = if level == 4 {
                    // 256TB 超页：PPN[3:0] 必须为 0 (低 36 位)
                    if (ppn & 0xFFFFFFFFF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0xFFFFFFFFFFFF) // 低 48 位
                } else if level == 3 {
                    // 512GB 超页：PPN[2:0] 必须为 0 (低 27 位)
                    if (ppn & 0x7FFFFFF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0x7FFFFFFFFF) // 低 39 位
                } else if level == 2 {
                    // 1GB 超页：PPN[1:0] 必须为 0 (低 18 位)
                    if (ppn & 0x3FFFF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0x3FFFFFFF) // 低 30 位
                } else if level == 1 {
                    // 2MB 超页：PPN[0] 必须为 0 (低 9 位)
                    if (ppn & 0x1FF) != 0 {
                        return Err(Self::page_fault(access_type, va));
                    }
                    (ppn << PAGE_SHIFT) | (va & 0x1FFFFF) // 低 21 位
                } else {
                    // 4KB 常规页
                    (ppn << PAGE_SHIFT) | page_offset
                };
                
                // 更新 A/D 位（硬件更新模式）
                let pte_a = (pte & PTE_A) != 0;
                let pte_d = (pte & PTE_D) != 0;
                let mut updated_pte = pte;
                let mut needs_update = false;
                
                // 1. 所有访问都需要设置 A 位
                if !pte_a {
                    updated_pte |= PTE_A;
                    needs_update = true;
                }
                
                // 2. Store 访问必须设置 D 位
                if access_type == AccessType::Store && !pte_d {
                    updated_pte |= PTE_D;
                    needs_update = true;
                }
                
                // 3. 如果需要更新，将新 PTE 写回页表
                if needs_update {
                    if let Err(_) = bus.write64(pte_addr, updated_pte) {
                        return Err(Self::page_fault(access_type, va));
                    }
                }
                
                return Ok(pa);
            } else {
                // 非叶子节点：继续遍历下一级
                let ppn = (pte >> 10) & 0x0FFF_FFFF_FFFF;
                pte_addr = (ppn << PAGE_SHIFT) as u64;
            }
        }
        
        // 不应该到达这里
        Err(Self::page_fault(access_type, va))
    }
    
    /// 权限检查
    fn check_permissions(
        pte: u64,
        access_type: AccessType,
        mode: Mode,
        mstatus: u64,
    ) -> Result<(), Trap> {
        let pte_r = (pte & PTE_R) != 0;
        let pte_w = (pte & PTE_W) != 0;
        let pte_x = (pte & PTE_X) != 0;
        let pte_u = (pte & PTE_U) != 0;
        
        // MXR (Make eXecutable Readable): mstatus bit 19
        let mxr = (mstatus >> 19) & 0x1 != 0;
        
        // SUM (permit Supervisor User Memory access): mstatus bit 18
        let sum = (mstatus >> 18) & 0x1 != 0;
        
        // 1. User 页面权限检查
        if pte_u {
            // User 页面只能在 User Mode 访问 (除非 SUM=1)
            // ⚠️ 关键修复：根据访问类型返回正确的 PageFault 类型
            if mode == Mode::Supervisor && !sum {
                return Err(Self::page_fault(access_type, 0));  // va 参数这里不用，page_fault 只看 access_type
            }
            // ⚠️ 额外检查：User 页面不能执行指令（除非 PTE.X=1）
            // 这里不需要特殊处理，后面的 R/W/X 检查会处理
        } else {
            // Supervisor 页面不能在 User Mode 访问
            // ⚠️ 关键修复：根据访问类型返回正确的 PageFault 类型
            if mode == Mode::User {
                return Err(Self::page_fault(access_type, 0));
            }
        }
        
        // 2. 根据访问类型检查 R/W/X 权限
        match access_type {
            AccessType::Instruction => {
                // ⚠️ 额外检查：Supervisor 模式不能执行 User 页面的代码（U=1 + X=1 仅限 User 执行）
                if pte_u && mode == Mode::Supervisor {
                    return Err(Trap::Exception(Exception::InstructionPageFault));
                }
                if !pte_x {
                    return Err(Trap::Exception(Exception::InstructionPageFault));
                }
            }
            AccessType::Load => {
                // MXR=1 时，可执行页也可读
                if !pte_r && !(mxr && pte_x) {
                    return Err(Trap::Exception(Exception::LoadPageFault));
                }
            }
            AccessType::Store => {
                if !pte_w {
                    return Err(Trap::Exception(Exception::StoreAMOPageFault));
                }
            }
        }
        
        Ok(())
    }
    
    /// 根据访问类型生成对应的页错误异常
    fn page_fault(access_type: AccessType, va: u64) -> Trap {
        match access_type {
            AccessType::Instruction => Trap::Exception(Exception::InstructionPageFault),
            AccessType::Load => Trap::Exception(Exception::LoadPageFault),
            AccessType::Store => Trap::Exception(Exception::StoreAMOPageFault),
        }
    }
}
