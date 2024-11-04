# Verilog RISC-V RV32I CPU

#### author:Zevick

## 基本描述

复旦2024春季，数字逻辑基础（H）课程作业，本项目仍在建设当中。

使用RISC-V指令集，利用Verilog描述一个简易的CPU结构，旨在实现过程中学习相应知识。

## 参考资料

* 《计算机组成与设计：硬件/软件接口》 ， David A. Patterson，John L. Hennessy，ISBN 978-7-111-72797-2
* 《CPU制作入门：基于RISC_V和Chisel》，西山悠太郎，井田健太，ISBN 978-7-03-076965-7
* 《RISC-V手册》，David Patterson，Andrew Waterman

## 计划清单

- [x] 超前进位加法器设计
- [ ] 测试验证加法器功能正确性
- [ ] 与行为级描述自动综合+等算子进行比较
- [x] 指令与常量宏定义
- [x] 简易memory
- [x] 非流水线core设计
- [ ] 测试验证单周期指令core设计正确性
- [ ] 插入寄存器，划分成经典5阶段流水线
- [ ] 简单验证
- [ ] 处理数据冒险与结构冒险问题
- [ ] 再次验证
- [ ] 详细了解汇编编程思路
- [ ] 下载RISC-V工具编译C程序，进一步深入了解
- [ ] 编写一些用于验证的汇编程序
- [ ] 写一些简单的库
- [ ] 学习汇编程序的文件读写，实现write功能
- [ ] 了解总线与数据传输
- [ ] 将VGA显示抽象成文件，在屏幕上以ASCII码显示结果
- [ ] 重新设计memory，支持8bit,16bit数据读写
- [ ] 具体设计icache与dcache

***

## 文件描述

|          文件名          |            基本内容             |
| :----------------------: | :-----------------------------: |
| carry_look-ahead_adder.v |         超前进位加法器          |
|      riscv32_alu.v       |  alu设计，可以尝试用行为级描述  |
|     riscv32_memory.v     | 存储单元设计，目前固定32bit读写 |
|     riscv32_Consts.v     | 控制部分要使用的一些常量宏定义  |
|  riscv32_Instructions.v  |         RV32I指令宏定义         |
|      riscv32_core.v      |           单周期core            |
|      riscv32_top.v       |         简单组装各模块          |
|        test_tb.v         |            测试文件             |
|                          |                                 |

## 设计思路

主要参考了《CPU制作入门：基于RISC_V和Chisel》的思想。考虑到可扩展性，代码大部分使用了宏定义，而不是直接显示截取func3, func7, opcode等指令片段来完成译码，控制信号也使用宏定义，这样的Verilog代码更容易理解与维护。