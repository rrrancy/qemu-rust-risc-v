riscv64-emulator/
├── src/
│   ├── main.rs            # 模拟器入口 | 主循环 (Fetch-Decode-Execute)
│   ├── bus.rs             # 系统总线 | MMIO 路由与地址解码
│   │
│   ├── [Core Architecture]
│   ├── cpu.rs             # CPU 核心 | RV64IMAFDC 指令集解释执行
│   ├── csr.rs             # 控制状态寄存器 | 特权级切换 (M/S/U Mode)
│   ├── mmu.rs             # 内存管理单元 | Sv39/Sv48/Sv57 页表遍历与 TLB
│   ├── trap.rs            # 异常与中断处理 | Exception/Interrupt 分发
│   │
│   ├── [Memory System]
│   ├── dram.rs            # 内存模拟 | 基于 mmap 的宿主机内存映射
│   ├── mrom.rs            # Boot ROM | 存放复位向量与设备树
│   ├── flash.rs           # CFI Flash | 模拟并行 Flash 存储
│   │
│   └── [Peripherals & I/O]
│       ├── plic.rs        # 中断控制器 | 平台级中断仲裁与分发
│       ├── uart.rs        # 串口通信 | 16550 UART 标准实现 (Console)
│       ├── virtio.rs      # 块设备驱动 | VirtIO-Blk Legacy MMIO 实现
│       └── virtio_rng.rs  # 随机数设备 | VirtIO-RNG 实现
│
├── [Firmware & Resources]
├── Cargo.toml             # Rust 项目依赖配置
├── riscv64-virt.dts/dtb   # 设备树源文件与二进制 (描述硬件拓扑)
├── mrom.bin / u-boot.bin  # 启动固件 (OpenSBI / U-Boot)
└── drive.img              # 虚拟磁盘镜像 (Debian Linux RootFS)