# Verilog RISC-V RV32I CPU

#### author:Zevick

## 基本描述

复旦2024秋季，数字逻辑基础（H）课程作业，本项目已成功通过Basys3上板验证。

使用RISC-V指令集，利用Verilog描述一个简易的CPU结构，旨在初步了解CPU的工作原理。

## 参考资料

* 《计算机组成与设计：硬件/软件接口》 ， David A. Patterson，John L. Hennessy，ISBN 978-7-111-72797-2
* 《CPU制作入门：基于RISC_V和Chisel》，西山悠太郎，井田健太，ISBN 978-7-03-076965-7
* 《RISC-V手册》，David Patterson，Andrew Waterman

## 主要特点

- [x] 指令与常量宏定义
- [x] 五级流水线RISC-V cpu设计
- [x] 利用RISC-V工具编译C程序，并成功运行
- [x] 利用VGA显示CPU具体工作状态

## 表现描述

通过Switch设置工作频率以及单步调试，单步调试模式下通过按Button步进。7段数码管显示当前周期数。VGA显示情况如下：

<p align = "center">    
<img  src="D:\Code_Files\Xilinx\tmp1\VGA\screenshot.png" width="300" />
</p>

***

## 文件描述

|          文件名           |            基本内容             |
| :----------------------: | :-----------------------------: |
|     riscv32_Consts.v     | 控制部分要使用的一些常量宏定义    |
|  riscv32_Instructions.v  |         RV32I指令宏定义         |
| riscv32_core_pipeline.v  |         五级流水core            |
|       Soc_top.v          |         简单组装各模块           |
|        testbench         |            测试文件             |
|         VGA              |          VGA字符显示模块        |

## 设计思路

主要参考了《CPU制作入门：基于RISC_V和Chisel》的代码设计。考虑到可扩展性，代码大部分使用了宏定义，而不是直接显式截取func3, func7, opcode等指令片段来完成译码，控制信号也使用宏定义，这样的Verilog代码更容易理解与维护。

## 欠缺

1. memory部分比较简陋，目前是每次将可执行程序烧录进ROM，意味着每次改变程序均要重新综合。
2. VGA显示为硬连线，原本可以用数据传输的方式用软件实现
3. 以上均体现了未考虑使用**通信协议**的而可能有的一系列严重后果