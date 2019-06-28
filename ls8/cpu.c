#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cpu.h"

#define DATA_LEN 6
#define SP 7

/**
 * Load the binary bytes from a .ls8 source file into a RAM array
 */
void cpu_load(struct cpu *cpu, char *filename)
{
  // char data[DATA_LEN] = {
  //   // From print8.ls8
  //   0b10000010, // LDI R0,8
  //   0b00000000,
  //   0b00001000,
  //   0b01000111, // PRN R0
  //   0b00000000,
  //   0b00000001  // HLT
  // };

  // int address = 0;

  // for (int i = 0; i < DATA_LEN; i++) {
  //   cpu->ram[address++] = data[i];
  // }

  int address = 0;
  FILE *fp;
  char line[1024];

  // Get file
  fp = fopen(filename, "r");


  // File error handling
  if (fp == NULL) {
    fprintf(stderr, "Error opening the file, or file not found");
    exit(1);
  }

  while (fgets(line, sizeof(line), fp) != NULL) {
    char *endptr;

    // Code from guided demo
    unsigned char v = strtoul(line, &endptr, 2);

    if (endptr == line) {
      continue;
    }

    cpu->ram[address] = v;
    address++;
  }

  fclose(fp);


  // TODO: Replace this with something less hard-coded
}

/**
 * ALU
 */
void alu(struct cpu *cpu, enum alu_op op, unsigned char regA, unsigned char regB)
{
  unsigned char value;
  switch (op) {
    case ALU_MUL:
      // TODO
      cpu->registers[regA] = cpu->registers[regA] * cpu->registers[regB];
      break;

    case ALU_ADD:
      // Temp value
      value = cpu->registers[regB];
      // Update vale
      value = value + cpu->registers[regA];
      // Set value to register
      cpu->registers[regA] = value;
      break;
    
    case ALU_CMP:
      // value comparisons
      if (cpu->registers[regA] == cpu->registers[regB]) {
        //Equal flag
        cpu->fl ^= 0b0000001;
      } else if (cpu->registers[regA] > cpu->registers[regB]) {
        // greater then flag
        cpu->fl ^= 0b0000010;
      } else {
        // less then flag
        cpu->fl ^= 0b0000100;
      }
      break;
      

    // TODO: implement more ALU ops
  }
}

/**
 * Read from RAM
 */

unsigned char cpu_ram_read(struct cpu *cpu, int index)
{
  return cpu->ram[index];
};

/**
 * Write to RAM
 */

void cpu_ram_write(struct cpu *cpu, unsigned char value, int index )
{
  cpu->ram[index] = value;
};

//
// HELPER FUNCTION
//


void handle_push(struct cpu *cpu, unsigned char regA) {

  //decrement stack pointer
  cpu->registers[SP]--;

  // get register value
  int value = cpu->registers[regA];


  // push to stack
  cpu_ram_write(cpu, value, cpu->registers[SP]);
}



void handle_pop(struct cpu *cpu, unsigned char regA)
{
  // read from stack pointer
  int value = cpu_ram_read(cpu, cpu->registers[SP]);

  //store value in register
  cpu->registers[regA] = value;

  // Increment stack pointer
  cpu->registers[SP]++;
}

void handle_ldi(struct cpu *cpu, unsigned char regA, unsigned char regB)
{
  // set reg at first point to second
  int reg_index = regA & 0b00000111;
  cpu->registers[reg_index] = regB;
}

void handle_prn(struct cpu *cpu, unsigned char regA)
{
  // Get register at first pint
	int reg_index = regA & 0b00000111;
  // Print it
  printf("%d\n", cpu->registers[reg_index]);
}

void handle_call(struct cpu *cpu, unsigned char regA)
{
  // decrement stack pointer
  cpu->registers[SP]--;
  //push instruction to stack
  cpu_ram_write(cpu, cpu->pc+2, cpu->registers[SP]);
  // set pc to register value
  cpu->pc = cpu->registers[regA];
}

void handle_ret(struct cpu *cpu)
{
  // pop instruction from top of stack and put in pc
	cpu->pc = cpu_ram_read(cpu, cpu->registers[SP]);
	
  // Increment stack pointer
	cpu->registers[SP]++;
}

void handle_hlt(int *running)
{
  *running = 0;
}
	
void handle_jeq(struct cpu *cpu, unsigned char regA, int op_count)
{
  // see if there is equal flag
	if (cpu->fl & 0b00000001) {
	
    // set pc to reg index
	  cpu->pc = cpu->registers[regA];
	
	} else {
	
    // set cpu->pc
	  cpu->pc += op_count + 1;
	}
}

void handle_jne(struct cpu *cpu, unsigned char regA, int op_count)
{
  // check if no equal flag
	if (!(cpu->fl & 0b00000001)) {
	
    // set pc to reg index
	  cpu->pc = cpu->registers[regA];
	
	} else {
	
    // set cpu->pc
	  cpu->pc += op_count + 1;
	}
}

void handle_jmp(struct cpu *cpu, unsigned char regA)
{
  cpu->pc = cpu->registers[regA];

}
/**
 * Run the CPU
 */
void cpu_run(struct cpu *cpu)
{
  int running = 1; // True until we get a HLT instruction

  unsigned char ir;
  unsigned char regA;
  unsigned char regB;
  int op_count;

  while (running) {
    // TODO
    // 1. Get the value of the current instruction (in address PC).
    ir = cpu_ram_read(cpu, cpu->pc);
    // 2. Figure out how many operands this next instruction requires
    op_count = ir>>6;
    // 3. Get the appropriate value(s) of the operands following this instruction
    regA = cpu_ram_read(cpu, cpu->pc + 1);
    regB = cpu_ram_read(cpu, cpu->pc + 2);
    // 4. switch() over it to decide on a course of action.
    switch(ir) {
      case CALL:
        handle_call(cpu, regA);
        break;

      case RET:
        handle_ret(cpu);
        break;

      case PUSH:
        handle_push(cpu, regA);
        break;

      case POP:
        handle_pop(cpu, regA);
        break;

      case LDI:
        handle_ldi(cpu, regA, regB);
        break;

      case PRN:
        handle_prn(cpu, regA);
        break;

      case MUL:
        alu(cpu, ALU_MUL, regA, regB);
        break;

      case ADD:
        alu(cpu, ALU_ADD, regA, regB);
        break;

      case CMP:
        alu(cpu, ALU_CMP, regA, regB);
        break;

      case JEQ:
        handle_jeq(cpu, regA, op_count);
        cpu->fl = 0;
        break;

      case JNE:
        handle_jne(cpu, regA, op_count);
        cpu->fl = 0;
        break;

      case JMP:
        handle_jmp(cpu, regA);
        break;

      case HLT:
        handle_hlt(&running);
        break;

    }

    if (!(ir & SETS_PC_DIRECT)) {
	      cpu->pc += op_count + 1;
	  }
    // 5. Do whatever the instruction should do according to the spec.
    // 6. Move the PC to the next instruction.
  }
}

/**
 * Initialize a CPU struct
 */
void cpu_init(struct cpu *cpu)
{
  // TODO: Initialize the PC and other special registers
  cpu->pc = 0;
  // init flags to 0
  cpu->fl = 0;

  memset(cpu->registers, 0, sizeof(cpu->registers));
  memset(cpu->ram, 0 , sizeof(cpu->ram));

  // stack pointer set to 244
  cpu->registers[SP] = 244;

}
