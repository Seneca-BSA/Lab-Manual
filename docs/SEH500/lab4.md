# Lab 4 : Branching, Array and String in Assembly

<font size="5">
Seneca Polytechnic</br>
SEH500 Microprocessors and Computer Architecture
</font>

## Introduction

Documentation of the Cortex-M4 instruction set can be found here:

- [Arm Cortex-M4 Processor Technical Reference Manual Revision](https://developer.arm.com/documentation/100166/0001) ([PDF](Cortex-M4-Proc-Tech-Ref-Manual.pdf))
    - [Table of processor instructions](https://developer.arm.com/documentation/100166/0001/Programmers-Model/Instruction-set-summary/Table-of-processor-instructions)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/) ([PDF](DDI0403E_e_armv7m_arm.pdf))

In the previous lab, we explored how to use the program status flags and how to perform branching. In this lab, we'll further explore branching and variables in the form of arrays and strings.

### Review of Branching Instructions

| Mnemonic | Meaning |
|---|---|
| B label  | Branch |
| B{cond} label | Branch with condition |
| BL label | Branch with Link |
| BL{cond} label | Branch with Link with condition |
| BX Rm  | Branch to register value |
| BX{cond} Rm | Branch to register value with condition |
| BLX Rm | Branch with Link to register value |
| BLX{cond} Rm| Branch with Link to register value with condition |

See below for conditions that are set by a compare, usually `CMP`, instruction. The `CMP Rm, Rn` instruction operates `Rm-Rn` for the sole purpose of setting the condition flags.

### Review of Branching Conditions

The APSR contains the following condition flags:

- **N**: Set to 1 when the result of the operation was negative, cleared to 0 otherwise.
- **Z**: Set to 1 when the result of the operation was zero, cleared to 0 otherwise.
- **C**: Set to 1 when the operation resulted in a carry, cleared to 0 otherwise.
- **V**: Set to 1 when the operation caused overflow, cleared to 0 otherwise.

| Suffix | Flags | Meaning |
|---|---|---|
| EQ | Z = 1 | Equal |
| NE | Z = 0 | Not equal |
| CS or HS | C = 1 | Higher or same, unsigned, >= |
| CC or LO | C = 0 | Lower, unsigned, < |
| MI | N = 1 | Negative |
| PL | N = 0 | Positive or zero
| VS | V = 1 | Overflow |
| VC | V = 0 | No overflow |
| HI | C = 1 and Z = 0 | Higher, unsigned, > |
| LS | C = 0 or Z = 1 | Lower or same, unsigned, <= |
| GE | N = V | Greater than or equal, signed, >= |
| LT | N != V | Less than, signed, < |
| GT | Z = 0 and N = V | Greater than, signed, > |
| LE | Z = 1 and N != V | Less than or equal, signed, <= |
| AL | Can have any value | Always. This is the default when no suffix is specified. |

### Examples of Compare Instructions

| Mnemonic | Meaning |
|---|---|
| CBZ R5, label | Compare and branch to label if R5 is zero |
| CBNZ R0, label | Compare and branch to label if R0 is not zero |
| CMP R2, R9 | Compare R2 - R9, update the N, Z, C and V flags<br/>Same as SUBS except the result is discarded |
| CMN R0, #64 | Compare (negative) R0 + #64, update the N, Z, C and V flags<br/>Same as ADDS except the result is discarded |

### Example of Branching Instructions

Following a compare instruction, branching instructions can be used as follows:

| Instruction | Action |
|---|---|
| B label | Branch to label unconditionally |
| BEQ label | Conditionally branch to label, when Z = 1 |
| BNE label | branch to label when Z = 0 |
| BMI label | branch to label when N = 1 |
| BPL label | branch to label when N = 0 |
| BLT label | Conditionally branch to label, when N is set and V is clear or N is clear and V is set<br/>i.e. N != V |
| BLE label | Conditionally branch to label, when less than or equal, <br/>Z is set or N is set and V is clear or N is clear and V is set<br/>i.e. Z = 1 or N != V |
| BGT  label | Conditionally branch to label, when Z is clear <br/>and either N is set and V is set or N is clear and V is clear<br/>i.e.   Z = 0 and N == V |
| BGE label | Conditionally branch to label, when greater than or equal to zero, <br/>Z is set or N is set and V is clear or N is clear and V is set<br/>i.e. Z = 1 or N == V |
| BL label | Branch with link (Call) to label, with return address stored in LR (register R14) |
| BX LR | Return from function call, loading LR into PC |
| BXNE R0 | Conditionally branch to address stored in R0 |
| BLX R0 | Branch with link (Call) to an address stored in R0,<br/>with return address stored in LR (register R14) |

### Register Addressing Mode

In addition to the basic LDR and STR, there are also a few other different ways to specify the address for load and store instructions.

| Name | Examples | Meaning |
|---|---|---|
| Register to register<br/>Register direct | MOV R0, R1 | |
| Absolute<br/>Direct | LDR R0, =address | |
| Literal<br/>Immediate | MOV R0, #15<br/>ADD R1, R2, #12 | |
| Indexed, base<br/>Register indirect | LDR R0, [R1] | Load R0<br/>with the word pointed by R1 |
| Pre-indexed, base with displacement<br/>Register indirect with offset | LDR R0, [R1, #4] | Load R0<br/>with the word pointed by<br/>R1+4 |
| Pre-indexed, autoindexing<br/>Register indirect pre-incrementing | LDR R0, [R1, #4]! | Load R0<br/>with the word pointed by<br/>R1+4 then add 4 to R1 |
| Post-indexing, autoindexed<br/>Register indirect post-increment | LDR R0, [R1], #4 | Load R0<br/>with the word pointed by R1<br/>then add 4 to R1 |
| Double Reg indirect<br/>Register indirect register indexed | LDR R0, [R1, R2] | Load R0<br/>with the word pointed by<br/>R1+R2 |
| Program counter relative | LDR R0, [PC, #offset] | Load R0<br/>with the word pointed by<br/>PC+offset |

### IT (If-Then) Block

The `IT` (If-Then) instruction makes a single following instruction (the IT block) conditional. In another words, the IT black helps simplify the If-Then logics without having to explicitly use branching.

**NOTE**: An IT block is not always required as some IDE will insert it for you automatically.

An IT block has the following syntax:

    IT cond
    instruction

Where:

- `cond` - specifies the condition for the first instruction in the IT block.
- `instruction` - specifies the instruciton (only one line) to execute for the true case.

**IT block covering more than one instruction is deprecated**

Example:

    it     ne			@ define the condition of the IT block
    andne  r0,r0,r1 	@ instruction to be executed only if TRUE
    add    r0,r0,r1		@ not in IT block, always executed

The trigger(flag) of the condition come from previous instruction such as `cmp`, `subs`, etc.

## Procedures

1. Open MCUXpresso then start a new C/C++ project based on the Freedom board model that you have.

1. In the new project configuration, rename the project then leave all other settings as default.

1. Once the project is created, rename the C-code file from ".c" to ".s". If the IDE is not allowing you to rename, delete the C-code file and create a new file of the same name but with the extension ".s".

1. In the previous lab, we also explored the idea of using a label to reference address in memory used in the same manner as variables in high-level programming language, this time, we'll explore the use of string. Replace the code within the file with the following:

        @ Example Code 4.1

        .syntax unified             @ unified syntax used
        .cpu cortex-m4              @ cpu is cortex-m4
        .thumb                      @ use thumb encoding

        .data                       @ put data in the data section
        myString: .string ""        @ put your name within the ""
            @ declare a label for a string with a null terminator
        
        .text
        .global main                @ declare main as a global variable
        .type main, %function       @ set main to function type

        main:
            ldr r0, =myString       @ Load the address of myString into R0
                                    @ R0 = ?
            mov r1, #0              @ Initialize the counter
                                    @ R1 = ?

        loopCount:
            ldrb r2, [r0]           @ Load the character from the address in R0
            cmp r2, #0              @ check for null terminator
                                    @ R2 = ? During the first loop
                                    @ R2 = ? At the end of the program
                                    @ Which ASCII characters are they?
            beq countDone           @ If it is zero...null terminated...
                                    @ We are done with the string.
                                    @ The length is in R1.
            add r0, #1              @ Otherwise, increment the
                                    @ address to the next character
                                    @ R0 = ? During the first loop
                                    @ R0 = ? At the end of the program
            add r1, #1              @ increment the counter for length
                                    @ R1 = ? During the first loop
                                    @ R1 = ? At the end of the program
                                    @ What is the length of the string?
            b loopCount

        countDone:
        stop:                           @ define a new label called stop
            nop                         @ do nothing
            b       stop                @ jump back label stop to form a loop

1. Execute the code above, pay attention to what is happening in each register and record their value as per the code comment. Afterward, take a look at the memory address where the string is saved then take a screenshot showing the memory address along with the value and ASCII character saved in it. Submit your code and screenshot as part of the lab question.

1. In the previous code, we explored the idea of using string. In this code, we'll take a look at the use of an array. Replace the code within the file with the following:

        @ Example Code 4.2

        .syntax unified             @ unified syntax used
        .cpu cortex-m4              @ cpu is cortex-m4
        .thumb                      @ use thumb encoding

        .data                       @ put data in the data section
        sump: .word sum             @ a pointer for sum
        pointer: .word num1         @ a pointer for num1
        n: .word 5                  @ variable n, word size, value 5
        num1: .word #, -#, -#, -#, # 
            @ replace it with the last 5 digits of your student number and maintain the negative
                                    @ array num1 with 5 elements, word size
        sum: .word 0                @ variable sum, word size, value 0
        
        .text
        .global main                @ declare main as a global variable
        .type main, %function       @ set main to function type

        main:
            ldr r1, =n
            ldr r1, [r1]            @ load size of array, R1 = ?
            @ a counter for how many elements are left to process
            ldr r2, =pointer
            ldr r2, [r2]            @ load base pointer of array, R2 = ?
            mov r0, #0              @ initialize counter, R0 = ?

        loop:   
            ldr r3, [r2], #4        @ load value from array
                                    @ R3 = ? for the first loop
                                    @ R3 = ? for the second loop
                @ increment array pointer to next word (index)
            add r0, r0, r3          @ add value from array to counter
                                    @ R0 = ? for the first loop
                                    @ R0 = ? for the second loop
            subs r1, r1, #1         @ decrement word counter
                                    @ R1 = ? for the first loop
                                    @ R1 = ? for the second loop
            bgt loop                @ keep looping until the counter is zero
            ldr r4, =sump
            ldr r4, [r4]            @ get memory address to store sum
            str r0, [r4]            @ store answer
            ldr r6, [r4]            @ Check the value in the sum

        stop:                           @ define a new label called stop
            nop                         @ do nothing
            b       stop                @ jump back label stop to form a loop

1. Execute the code above, pay attention to what is happening in each register and record their value as per the code comment. Afterward, take a look at the memory address where the array is saved then take a screenshot showing the memory address along with the value and highlight the negative. Submit your code and screenshot as part of the lab question.

1. Lastly, the code from the first example can also be written as follows using an array instead of a string. Also, a different comparison statement is used.

        @ Example Code 4.3
        
        .syntax unified             @ unified syntax used
        .cpu cortex-m4              @ cpu is cortex-m4
        .thumb                      @ use thumb encoding

        .data                       @ put data in the data section
        myString: .ascii "\0"       @ put your name before \0
            @ declare a label for a string with a null terminator
        
        .text
        .global main                @ declare main as a global variable
        .type main, %function       @ set main to function type

        main:
            ldr r0, =myString       @ Load the address of myString into R0
            mov r1, #0              @ Initialize the counter

        loopCount:
            ldrb r2, [r0], #1       @ Load the character from the address in R0
                                    @ and update the pointer R0
            cbz r2, countDone       @ check for null terminator in one line
                                    @ If it is zero...null terminated...
                                    @ We are done with the string.
                                    @ The length is in R1.
            add r1, #1              @ increment the counter for length
            b loopCount

        countDone:
        stop:                           @ define a new label called stop
            nop                         @ do nothing
            b       stop                @ jump back label stop to form a loop

1. Compare the third example code with the first example code and find the difference between then explain which line got added, removed, changed and why. Also, discuss its effect on the code then put your answer in the lab question.

## Lab Questions

<div style="padding: 15px; border: 1px solid orange; background-color: orange; color: black;">
<b>GenAI Usage Policy:</b>
<p>Submission of answers or code generated by AI, or from external help, containing concepts or instructions not shown/taught/discussed in this course will immediately result in a report of Academic Integrity violation.</p>
<p>If GenAI was used as a learning aid, you must cite it for every answer that used GenAI as a tool, as follows:<p>
<ul><li>GenAI used: Copilot for concept research; ChatGPT for editing and grammar enhancement; Gemini for code generation in section 1, as identified.</li></ul>
</div>

Using the skills and knowledge acquired from this lab, answer the following post-lab question(s) on Blackboard. Due one week after the lab.

1. Execute Example Code 4.1, then answer the questions in the comment. Copy and paste your code with your answers, along with the necessary screenshots, into Blackboard. **No mark will be awarded if no screenshot with the expected result is provided.**

1. Which line(s) in Example Code 4.1 are responsible for looping through the char array, and explain how it works? Do not include line(s) that are irrelevant to looping.

1. Execute Example Code 4.2, then answer the questions in the comment. Copy and paste your code with your answers, along with the necessary screenshots, into Blackboard. **No mark will be awarded if no screenshot with the expected result is provided.**

1. Which line(s) in Example Code 4.2 are responsible for looping through the number array, and explain how it works? Do not include line(s) that are irrelevant to looping.

1. How are Example Code 4.3 and Example Code 4.1 different in terms of loop(s) and comparison statement(s)?

1. Write a program in assembly language that counts how many vowels and non-vowels (including space, symbol, etc.) are in "SEH500 is very cool! (your name here)" (REPLACE (your name here) with your name).

    - Hint: put your string into memory using the .string directive
    - use R0 to hold the address for the string or character
    - use R1 as the counter for vowel
    - use R2 as the counter for non-vowel

    For the code above, at the end of execution, take a screenshot of your register bank and your memory space, highlighting the string, then copy your code into Blackboard. **No mark will be awarded if no screenshot with the expected result is provided.**

## Reference

[1] Yiu, J. (2013). The Definitive Guide to ARM® Cortex®-M3 and Cortex®-M4 Processors. (3rd ed.). Elsevier Science & Technology.