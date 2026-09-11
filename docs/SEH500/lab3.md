# Lab 3 : Branching in Assembly

<font size="5">
Seneca Polytechnic</br>
SEH500 Microprocessors and Computer Architecture
</font>

## Introduction

Documentation of the Cortex-M4 instruction set can be found here:

- [Arm Cortex-M4 Processor Technical Reference Manual Revision](https://developer.arm.com/documentation/100166/0001) ([PDF](Cortex-M4-Proc-Tech-Ref-Manual.pdf))
    - [Table of processor instructions](https://developer.arm.com/documentation/100166/0001/Programmers-Model/Instruction-set-summary/Table-of-processor-instructions)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/) ([PDF](DDI0403E_e_armv7m_arm.pdf))

As you've seen in the previous lab, the ARM processor has a Program Status Register with 4 flags that might get set or clear depending on the previous ALU operation.

- `N`: Negative flag - set to 1 when the result of an operation is negative
- `Z`: Zero flag - set to 1 when the result of an operation is zero
- `C`: Carry (or NOT borrow) flag
    - set to 1 if the addition produces a carry
    - set to 0 if the subtraction produces a borrow
- `V`: Overflow flag - set to 1 if the result of an operation is outside the range of a signed 32-bit integer

These flags can then be used for decision-making within the program.

Below are instructions that might set or clear the status flag.

### Arithmetic Instructions

| Instruction | Mnemonic | Meaning |
|---|---|---|
| Addition | `ADD  R0, R1, R2` | R0 = R1 + R2 |
| Addition | `ADDS R0, R1, R2` | R0 = R1 + R2, and FLAGs are updated |
| Subtraction | `SUB  R1, R2, R3` | R1 = R2 - R3 |
| Subtraction | `SUBS R1, R2, R3` | R1 = R2 - R3, and FLAGs are updated |
| Subtraction | `SUBS R7, R6, #20` | R7 = R6 - 20, and sets the FLAGs on the result |
| Reverse Subtraction | `RSB  R4, R4, #120` | R4 = 120 - R4 |
| Multiply | `MUL  R0, R1, R2` | R0 = R1 * R2 |
| Division | `SDIV R0, R2, R4` | Signed divide, R0 = R2/R4 |
| Division | `UDIV R8, R8, R1` | Unsigned divide, R8 = R8/R1 |

### Move Instructions

| Mnemonic | Meaning |
|---|---|
| `MOV  R1, #0xFA05` | Write value of 0xFA05 to R1, flags are not updated |
| `MOVS R11, #0x000B` | Write value of 0x000B to R11, flags get updated |
| `MOVS R10, R12` | Write value in R12 to R10, flags get updated |
| `MOV  R3, #23` | Write value of 23 to R3 |
| `MOV  R8, SP` | Write value of stack pointer to R8 |
| `MVNS R2, #0xF` | Write value of 0xFFFFFFF0 (bitwise inverse of 0xF), to the R2 and update flags. |

### Logical Operation Instructions

| Mnemonic | Meaning |
|---|---|
| `AND   R9, R2, R1` | R9 = R2 AND R1 |
| `AND   R9, R2, #0xFF00` | R9 = R2 AND #0xFF00 |
| `ANDS  R9, R8, #0x19` | with flags update |
| `ORR   R9, R2, R1` | R9 = R2 OR R1 |
| `ORR   R9, R2, #0xFF00` | |
| `ORREQ R2, R0, R5` | |
| `EOR   R7, R11, R10` | R7 = R11 XOR R10 |
| `EORS  R7, R11, #0x18181818` | with flags update |
| `BIC   R0, R1, #0xab` | R0 = R1 AND (NOT(#0xab)) |
| `ORN   R7, R11, R14, ROR #4` | R7 = R11 OR (NOT(R14 ROR #4)) |
| `ORNS  R7, R11, R14, ROR #2` | with flags update |

### Shift Instructions

| Mnemonic | Meaning |
|---|---|
| `LSL  R4, R5, #2` | Logical shift left by 2 bits |
| `LSR  R4, R5, #6` | Logical shift right by 6 bits |
| `LSLS R1, R2, #3` | Logical shift left by 3 bits with flags update |
| `ROR  R4, R5, R6` | Rotate right by the value in the bottom byte of R6 |
| `RRX  R4, R5` | Rotate right with extend (one bit only). |

### Branching Instructions

| Mnemonic | Meaning |
|---|---|
| `B <label>` | Branch |
| `B{cond} <label>` | Branch with condition |
| `BL <label>` | Branch with Link |
| `BL{cond} <label>` | Branch with Link with condition |
| `BX <Rm>` | Branch to register value |
| `BX{cond} <Rm>` | Branch to register value with condition |
| `BLX <Rm>` | Branch with Link to register value |
| `BLX{cond} <Rm>` | Branch with Link to register value with condition |

See below for conditions that are set by a compare, usually by using the CMP, instruction. The `CMP Rm, Rn` instruction operates "Rm - Rn" for the sole purpose of setting the condition flags. Unlike a subtraction operation, the result of the compare operation is discarded.

### Branching Conditions

The APSR contains the following condition flags:

- `N`: Set to 1 when the result of the operation was negative, cleared to 0 otherwise.
- `Z`: Set to 1 when the result of the operation was zero, cleared to 0 otherwise.
- `C`: Set to 1 when the operation resulted in a carry, cleared to 0 otherwise.
- `V`: Set to 1 when the operation caused overflow, cleared to 0 otherwise.

Depending on how you want branching to be done, the condition suffix can be added to the branching instruction to form a conditional branching instruction.

| Suffix {cond} | Flags | Meaning |
|---|---|---|
| EQ |  Z = 1 |     Equal |
| NE |  Z = 0 |     Not equal |
| CS or HS |    C = 1 |     Higher or same, unsigned |
| CC or LO |    C = 0 |     Lower, unsigned |
| MI |  N = 1 |     Negative |
| PL |  N = 0 |     Positive or zero
| VS |  V = 1 |     Overflow |
| VC |  V = 0 |     No overflow |
| HI |  C = 1 and Z = 0 |   Higher, unsigned |
| LS |  C = 0 or Z = 1 |    Lower or same, unsigned |
| GE |  N = V |     Greater than or equal, signed |
| LT |  N != V |    Less than, signed |
| GT |  Z = 0 and N = V |   Greater than, signed |
| LE |  Z = 1 and N != V |  Less than or equal, signed |
| AL |  Can have any value |    Always. This is the default when no suffix is specified. |

### Variables

When defining a variable, the compiler assigns an address in memory with sufficient size to store the data. All variables are defined in the `.data` section of the code that should be placed above the `.text` section.

Some possible data types are:

- `.byte`: 1 byte
- `.short`: 2 bytes
- `.word`: 4 bytes
- `.quad`: 8 bytes
- `.octa`: 16 bytes

### Endianness (Data Arrangment)

Endianness describes the order in which bytes of multi-byte data (like integers) are stored in computer memory or transmitted over a network. The two main types are little-endian, which stores the least-significant byte first (at the lowest address), and big-endian, which stores the most-significant byte first. This order matters because if two systems with different endianness exchange data, the data can be misinterpreted.

Types of Endianness:

- **Little-Endian:** The least-significant byte (the "little end") is stored first, at the lowest memory address.
- **Big-Endian:** The most-significant byte (the "big end") is stored first, at the lowest memory address. This is also known as "network byte order" and is used in network protocols.

For example, consider the 32-bit hexadecimal number 0x12345678:

- **Little-Endian:** Stored in memory as 0x78 0x56 0x34 0x12
- **Big-Endian:** Stored in memory as 0x12 0x34 0x56 0x78

## Procedures

1. Open MCUXpresso then start a new C/C++ project based on the Freedom board model that you have.

1. In the new project configuration, rename the project then leave all other settings as default.

1. Once the project is created, rename the C-code file from ".c" to ".s". If the IDE is not allowing you to rename, delete the C-code file and create a new file of the same name but with the extension ".s".

1. Replace the code within the file with the following and fill in any missing information as required:

        @ Example Code 3.1
        
        .syntax unified             @ unified syntax used
        .cpu cortex-m4              @ cpu is cortex-m4
        .thumb                      @ use thumb encoding

        .global main                @ declare main as a global variable
        .type main, %function       @ set main to function type

        main:                           @ start of main code with a label
            mov r0, #                   @ r0 = <last digit of your student ID. use 1 if 0.>
            mul r1, r0, r0              @ r1 = ? and psr value N, Z, C, V = ?
                                        @ (ie. r1 = 5 and N, Z, C, V = 0, 0, 1, 0)
                                        @ what's math formula to get the value of r1?
                                        @ record all value after instruction execution
            mov r4, #5
            muls r1, r1, r4             @ r1 = ? and psr value N, Z, C, V = ?
                                        @ what's math formula to get the value of r1?
            mov r5, #6
            mul r2, r0, r5              @ r2 = ? and psr value N, Z, C, V = ?
                                        @ what's math formula to get the value of r2?
            subs r3, r1, r2             @ r3 = ? and psr value N, Z, C, V = ?
                                        @ what's math formula to get the value of r3?
            add r3, r3, #8              @ r3 = ? and psr value N, Z, C, V = ?
                                        @ what's math formula to get the value of r3?

        stop:                           @ define a new label called stop
            nop                         @ do nothing
            b       stop                @ jump back label stop to form a loop

1. Execute the code above and pay attention to what is happening in each register then record the PSR flags after each arithmetic instruction. Submit it as part of the lab question.

1. Below is another example but this time with variables and shifting. Although variables cannot be declared in the same way as a high-level programming language, it is possible to tell the assembler to automatically assign an address with a label and always reference it using the same label. Replace the code within the file with the following and fill in any missing information as required:

        @ Example Code 3.2
        
        .syntax unified             @ unified syntax used
        .cpu cortex-m4              @ cpu is cortex-m4
        .thumb                      @ use thumb encoding

        .data                       @ define the data section, used for variables
        num:    .word #             @ declare two word size labels (fill in the values)
        sum:    .word #             @ assign num and sum with values of 5 and 0

        .text                       @ define the text section, used for code
        .global main                @ declare main as a global variable
        .type main, %function       @ set main to function type

        main:                       @ start of main code with a label
            ldr r1, =num            @ Load count into r1
            ldr r1, [r1]            @ Load count into r1
            mov r0, #0              @ Clear accumulator r0

        loop:                       @ loop label for branching
            add r0, r0, r1          @ Add number into r0, @ record psr value for each loop
                                    @ after loop iteration 1, N, Z, C, V = ?
            subs r1, r1, #1         @ Decrement loop counter R1, @ record psr value for each loop
                                    @ after loop iteration 1, N, Z, C, V = ?
            bgt loop                @ Branch back if not done
            ldr r3, =sum            @ Load address of SUM to r3
            str r0, [r3]            @ Store SUM
            ldr r4, [r3]

        stop:                           @ define a new label called stop
            nop                         @ do nothing
            b       stop                @ jump back label stop to form a loop

1. Start the code in debug mode but don't run the code yet. Open up the memory view and start monitoring memory address `0x20000000`. You should see `00000005` at address `0x20000000`, which is the variable `num`. Right click in memory view and change the cell size to 1 byte. Refer to the Endianness discussion in the lab intro to determine if the data is stored as Little-endian or Big-endian in memory. Afteward, change the view back to 4 bytes for easier reading.

1. Execute the code above, pay attention the what is happening in each register then record the PSR flags after each arithmetic instruction and submit it as part of the lab question.

## Lab Questions

<div style="padding: 15px; border: 1px solid orange; background-color: orange; color: black;">
<b>GenAI Usage Policy:</b>
<p>Submission of answers or code generated by AI, or from external help, containing concepts or instructions not shown/taught/discussed in this course will immediately result in a report of Academic Integrity violation.</p>
<p>If GenAI was used as a learning aid, you must cite it for every answer that used GenAI as a tool, as follows:<p>
<ul><li>GenAI used: Copilot for concept research; ChatGPT for editing and grammar enhancement; Gemini for code generation in section 1, as identified.</li></ul>
</div>

Using the skills and knowledge acquired from this lab, answer the following post-lab question(s) on Blackboard. Due one week after the lab.

1. Execute the code from Example Code 3.1, then record the PSR flags. Copy and paste your code with PSR flags and a screenshot of the register bank (showing all the registers used) at the end of code execution into Blackboard for submission. **No mark will be awarded if no screenshot with the expected result is provided.**

1. Execute the code from Example Code 3.2, then record the PSR flags. Copy and paste your code with PSR flags and a screenshot of the register bank (showing all the registers used) at the end of code execution into Blackboard for submission. **No mark will be awarded if no screenshot with the expected result is provided.**

1. Write a program that converts Celsius to Fahrenheit or from Fahrenheit to Celsius depending on the input. If the input is above 32, it will assume the value is Fahrenheit and convert it to Celsius. If not, it will assume the value is Celsius and convert it to Fahrenheit.

    You will provide the input as data saved in R0. No user input (ie. scanf) is required. The input data will be the last two digits of your student number. Do NOT modify R0 afterward. Your code must be written in assembly with the output saved in a variable (labelled space in memory). Your code must consider both cases and determine which formula to use depending on the input (ie. the code should function with input above or below 32). You can use the following as your conversion equation. You can assume the input data will always be a positive integer.

    - C = 5 * (F - 32) / 9    
    - F = (9 * C / 5) + 32

    Put your name and student number as comments in the code, then copy and paste your code into Blackboard. Also, take a screenshot of your register bank as well as your memory space, highlighting the final variable value after code execution and paste them into Blackboard as well. **No mark will be awarded if no screenshot with the expected result is provided.**

## Reference

[1] Yiu, J. (2013). The Definitive Guide to ARM® Cortex®-M3 and Cortex®-M4 Processors. (3rd ed.). Elsevier Science & Technology.