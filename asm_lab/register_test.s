@ ARM processors have two main types of instructions: 
@   - ARM (32-bit)
@   - Thumb (16- or 32-bit)
@ Thumb is used on Cortex-M MCUs. It's more memory-efficient and suitable for small microcontrollers.
@ Without .syntax unified, the assembler might interpret instructions using the older ARM syntax, 
@ which can cause errors for Thumb instructions.

.syntax unified      @ Use unified ARM/Thumb syntax

@ Specify which CPU this program is written for
.cpu cortex-m3       @ Cortex-M3 specific instructions and registers

@ Cortex-M processors always use Thumb instructions (16/32-bit)
.thumb               @ Generate Thumb code

@ Define the entry point of the program
@ .global makes the label visible to the linker so it can set it as the start address in the ELF
.global _start


@ All Cortex-M based STM32 MCUs have the same general registers R0 -> R15
@ R0 --> R12: Can be used freely by the program (each register can only hold one value at a time)
@ R13 (SP): stack pointer → points to the top of the stack (stack grows downwards)
@           Stack = a memory “pile”
@           PUSH = add data on top (SP moves down)
@           POP  = remove top data (SP moves up)
@ R14 (LR): link register → stores the address to return to after a function
@ R15 (PC): program counter → points to the next instruction the CPU will execute

@ Main Program
_start:

    @ C code:    int num = 5;    looks something like this:
    MOV R0, #5      @ Temp store the value 5
    PUSH R0         @ Add to stack

    @ Stack is LIFO (Last In, First Out)
    @ If we store another value in R0 and PUSH again,
    @ popping will return values in reverse order of pushing

    @ If you do something like this:

    MOV R0, #1
    PUSH R0
    MOV R0, #2
    PUSH R0
    MOV R0, #3
    PUSH R0

    @ The stack now looks like this: 3, 2, 1
    @ So popping would return 3
    @ But what if you want to access the first value?
    @ You can use SP as a pointer and add an offset:

    LDR R2, [SP, #8] @ access the first value (assuming 3 pushes, 32-bit each) Each push of a 32-bit register moves SP down 4 bytes

    @ LDR stands for Load Register and is used to read a value from memory into a register
    
    @ PUSH saves one or more registers on the stack and adjusts SP automatically
    @ STR can also be used to write a value to a specific memoryaddress. Here you have to adjust SP manually like this:

    MOV R0, #5
    SUB SP, SP, #4      @ Make room on the stack. (SP = SP - 4). Point SP to the byte located under current SP position in memory
    STR R0, [SP]        @ Write R0:s value to the top of the stack.

    @ How to set a bit in a register:
    LDR R0, =0x40021018         @ RCC_APBENR address 
    LDR R1, [R0]            
    ORR R1, R1, #0x10       @ In C:  R1 |= (0x10)       @ This will set bit 4 in the RCC_APBENR register
    STR R1, [R0]            @ Save changes