# Lab 5 : More Branching, Subroutine, and Stack

<font size="5">
Seneca Polytechnic</br>
SEH500 Microprocessors and Computer Architecture
</font>

## Introduction

Documentation of the Cortex-M4 instruction set can be found here:

- [Arm Cortex-M4 Processor Technical Reference Manual Revision](https://developer.arm.com/documentation/100166/0001) ([PDF](Cortex-M4-Proc-Tech-Ref-Manual.pdf))
    - [Table of processor instructions](https://developer.arm.com/documentation/100166/0001/Programmers-Model/Instruction-set-summary/Table-of-processor-instructions)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/) ([PDF](DDI0403E_e_armv7m_arm.pdf))

In the previous lab, we explored how to use basic branching to control the flow of a program. In this lab, we'll further explore branching and use the stack point to create subroutine and function call procedure.

### Review of Branching Instructions with Link

| Mnemonic | Meaning |
|---|---|
| B label | Branch |
| B{cond} label | Branch with condition |
| BL label | Branch with Link |
| BL{cond} label | Branch with Link with condition |
| BX Rm | Branch to register value |
| BX{cond} Rm | Branch to register value with condition |
| BLX Rm | Branch with Link to register value |
| BLX{cond} Rm | Branch with Link to register value with condition |

See below for conditions that are set by a compare, usually `CMP`, instruction. The `CMP Rm, Rn` instruction operates `Rm-Rn` for the sole purpose of setting the condition flags.

### Review of Branching Conditions

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

### Register Use in the ARM Procudure Call Standard

So far, we've been using the general purpose registers freely without much condition of what there are ideal for or any standards. However, below is a common convention to consider when using registers:

| Purpose | Register |
|---|---|
| Argument 1 | r0 |
| Argument 2 | r1 |
| Argument 3 | r2 |
| Argument 4 | r3 |
| Variable 1 | r4 |
| Variable 2 | r5 |
| Variable 3 | r6 |
| Variable 4 | r7 |
| Variable 5 | r8 |
| Variable 6 / Stack Base | r9 |
| Variable 7 / Stack Limit | r10 |
| Variable 8 / Frame Pointer | r11 |
| Intra-Procedure Scratch | r12 |
| Stack Pointer (SP) | r13 |
| Link Register (LR) | r14 |
| Program Counter (PC) | r15 |

Note that even though SP, LR and PC are special register used by the processor, they can still be modified with code and used the same way as other registers.

### Introduction to Stack

The stack is a data structure, known as last in first out (LIFO). In a stack, items entered at one end and leave in the reversed order. Stacks in microprocessors are implemented by using a stack pointer to point to the top of the stack in memory.

As items are added to the stack (pushed), the stack pointer is moving up, and as items are removed from the stack (pulled or popped), the stack pointer is moved down.

ARM stacks are very flexible since the implementation is completely left to the software. Stack pointer is a register that points to the top of the stack. In the ARM processor, any one of the general purpose registers could be used as a stack pointer. Since it is left to the software to implement a stack, different implemenation choices result different types of stacks. Normally, there are two types of the stacks depending on which way the stack grows.

1.	**Ascending Stack** - When items are pushed on to the stack, the stack pointer is increasing.

    - That means the stack grows towards higher address.

1.	**Descending Stack** - When items are pushed on to the stack, the stack pointer is decreasing.

    - That means the stack is growing towards lower address.

Depending on what the stack pointer points to we can categorize the stacks into the following two types:

1.	**Empty Stack** - Stack pointer points to the location in which the next/first item will be stored. 
	
    - e.g. A push will store the value, and increment the stack pointer for an Ascending Stack.

2. 	**Full Stack** - Stack pointer points to the location in which the last item was stored. 
	
    - e.g. A pop will decrement the stack pointer and pull the value for an Ascending Stack.

So now we can have four possible types of stacks. They are:

1. full-ascending stack,
2. full-descending stack,
3. empty-ascending stack,
4. empty-descending stack.

They can be implemented by using the register load and store instructions.

Here are some instructions used to deal with stack:

    PUSH {R3}             @ Push R3 onto the stack
    PUSH {R0, R4-R7}	  @ Push R0, R4, R5, R6, R7 onto the stack
    PUSH {R2, LR}		  @ Push R2 and the link register onto the stack
    POP  {R0, R5, PC}	  @ pop and return from subroutine, branch to PC
    POP  {R3}             @ Pop stack value and save to R3

Push registers onto and pop registers off a full-descending stack.

### Subroutine and Stack

A subroutine call can be implemented by pushing the return address on the stack and then jumping to the branch target address. When the subroutine is done, remember to pop out the saved information so that it will be able to return to the next instruction immediately after the calling point.

## Procedures

1. Open MCUXpresso then start a new C/C++ project based on the Freedom board model that you have.

1. In the new project configuration, rename the project then leave all other settings as default.

1. Once the project is created, rename the C-code file from ".c" to ".s". If the IDE is not allowing you to rename, delete the C-code file and create a new file of the same name but with the extension ".s".

1. In this example, let's take a look as a simple subroutine call. Replace the code within the file with the following:

        @ Example Code 5.1
        .syntax unified             @ unified syntax used
        .cpu cortex-m4              @ cpu is cortex-m4
        .thumb                      @ use thumb encoding

        .data                       @ put data in the data section
        sump:   .word sum           @ declare a pointer to sum
        sump2:  .word sum2          @ declare a pointer to sum2
        n:      .word 5             @ declare variable n with value of 5
        sum:    .word 0             @ declare sum with value of 0
        sum2:   .word 0             @ declare sum2 with value of 0
        
        .text                       @ put code in the code section

        .global sumup               @ declare sumup as a global variable
        .type sumup, %function      @ set sumup to function type

        sumup:
            add 	r0, r0, r1 	    @ R0 = ? after first execution only
            subs 	r1, r1, #1      @ R1 = ? after first execution only
            bgt 	sumup           @ Branch back if not done
            bx      lr              @ PC = ?, LR = ?
                                    @ After execution, PC = ?

        .global main                @ declare main as a global variable
        .type main, %function       @ set main to function type

        main:
            ldr 	r1, =n 		    @ Load count into R1
            ldr 	r1, [r1]        @ R1 = ?
            mov 	r0, #0          @ Clear accumulator R0
            bl  	sumup            @ PC = ?, LR = ?
            ldr 	r3, =sump       @ Load address of SUM to R3
            ldr     r3, [r3]        @ R3 = ?
            str 	r0, [r3]	    @ Store SUM
            ldr     r4, [r3]        @ R4 = ?
            mov     r7, #8
            ldr 	r5, =sump2      @ Load address of SUM2 to R5
            ldr     r5, [r5]        @ R5 = ?
            str 	r7, [r5]	    @ Store SUM2
            ldr     r6, [r5]        @ R6 = ?

        stop:                           @ define a new label called stop
            nop                         @ do nothing
            b       stop                @ jump back label stop to form a loop

1. Execute the code above, pay attention to what is happening in each register and record their value as per the code comment. Afterward, take a look at the memory address at where the variable is saved then take a screenshot showing the memory address along with its data. Submit your code and screenshot it as part of the lab question.

1. In this code, we'll take a look at subroutine and stack. Replace the code within the file with the following:

        @ Example Code 5.2
        .syntax unified             @ unified syntax used
        .cpu cortex-m4              @ cpu is cortex-m4
        .thumb                      @ use thumb encoding

        .data                       @ put data in the data section
        sump:   .word sum           @ declare a pointer to sum
        sump2:  .word sum2          @ declare a pointer to sum2
        n:      .word 5             @ declare variable n with value of 5
        sum:    .word 0             @ declare sum with value of 0
        sum2:   .word 0             @ declare sum2 with value of 0
        
        .text                       @ put code in the code section

        .global function1           @ declare as a global variable
        .type function1, %function  @ set to function type

        function1:
            push    {r5, lr}        @ Save values in the stack
                                    @ which address did R5 and LR got saved to?
            mov     r5, #8          @ Set initial value for the delay loop	
        delay:
            subs	r5, r5, #1      @ R5 = ? after first execution
            bne     delay
            pop     {r5, pc}        @ pop out the saved value from the stack, 
                                    @ check the value in the R5
                                    @ and see if it is the saved value
                                    @ R5 = ?, PC = ?

        .global main                @ declare main as a global variable
        .type main, %function       @ set main to function type

        main:
            mov 	r0, #1          @ SP = ? then memory monitor to SP address
            mov 	r3, #0x75
            push	{r0, r3}        @ SP = ? which address did #0x75
                                    @ and #1 got saved to?
            mov 	r0, #6          @ R0 = ?
            mov 	r3, #7          @ R3 = ?
            pop     {r0, r3}        @ after pop, R0 = ?, R3 = ?, SP = ?
                                    @ did the value in memory address changed

        loop:
            add     r0, r0, #1
            cmp	    r0, #5
            bne     loop
            mov 	r5, #9          @ prepare for function call
            bl 	    function1       @ PC = ?, LR = ?, R5 = ?
            mov 	r3, #12

        stop:                           @ define a new label called stop
            nop                         @ do nothing
            b       stop                @ jump back label stop to form a loop

1. Execute the code above, pay attention to what is happening in each register and record their value as per the code comment. Afterward, take a look at the memory address at where the variable is saved then take a screenshot showing the memory address along with its data. Submit your code and screenshot it as part of the lab question.

## Lab Questions

<div style="padding: 15px; border: 1px solid orange; background-color: orange; color: black;">
<b>GenAI Usage Policy:</b>
<p>Submission of answers or code generated by AI, or from external help, containing concepts or instructions not shown/taught/discussed in this course will immediately result in a report of Academic Integrity violation.</p>
<p>If GenAI was used as a learning aid, you must cite it for every answer that used GenAI as a tool, as follows:<p>
<ul><li>GenAI used: Copilot for concept research; ChatGPT for editing and grammar enhancement; Gemini for code generation in section 1, as identified.</li></ul>
</div>

Using the skills and knowledge acquired from this lab, answer the following post-lab question(s) on Blackboard. Due one week after the lab.

1. Execute Lab Example Code 5.1, then answer the questions in the comment. Copy and paste your code with your answers, along with the necessary screenshots, into Blackboard. **No mark will be awarded if no screenshot with the expected result is provided.**

1. Execute Lab Example Code 5.2, then answer the questions in the comment. Copy and paste your code with your answers, along with the necessary screenshots, into Blackboard. **No mark will be awarded if no screenshot with the expected result is provided.**

1. Which one of the four types of stack are you using in Example Code 5.2? Justify your answer with the stack pointer and memory space. **Use screenshot(s) to support your answer.**

1. Rewrite the program from [lab 4](lab4.md) that counts how many vowels and non-vowels are in "SEH500 is very cool! (your name here)" (REPLACE (your name here) with your name). But this time, you must use at least one subroutine in your code along with the stack.

    - Hint: put your string into memory using the .string directive
    - use R0 to hold the address for the string or character
    - use R1 as the counter for the vowel
    - use R2 as the counter for non-vowel

    For the code above, at the end of execution, take a screenshot of your register bank and your memory space, highlighting the string, then copy your code into Blackboard. **No mark will be awarded if no screenshot with the expected result is provided.**

## Reference

[1] Yiu, J. (2013). The Definitive Guide to ARM® Cortex®-M3 and Cortex®-M4 Processors. (3rd ed.). Elsevier Science & Technology.