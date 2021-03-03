/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Nuvoton NPCM8xx Clock Generator binding
 * clock binding number for all clocks supportted by nuvoton,npcm7xx-clk
 *
 * Copyright (C) 2020 Nuvoton Technologies tomer.maimon@nuvoton.com
 *
 */

#ifndef __DT_BINDINGS_CLOCK_NPCM8XX_H
#define __DT_BINDINGS_CLOCK_NPCM8XX_H


#define NPCM8XX_CLK_CPU		0
#define NPCM8XX_CLK_GFX_PIXEL	1
#define NPCM8XX_CLK_MC		2
#define NPCM8XX_CLK_ADC		3
#define NPCM8XX_CLK_AHB		4
#define NPCM8XX_CLK_TIMER	5
#define NPCM8XX_CLK_UART	6
#define NPCM8XX_CLK_UART2	7
#define NPCM8XX_CLK_MMC		8
#define NPCM8XX_CLK_SPI3	9
#define NPCM8XX_CLK_PCI		10
#define NPCM8XX_CLK_AXI		11
#define NPCM8XX_CLK_APB4	12
#define NPCM8XX_CLK_APB3	13
#define NPCM8XX_CLK_APB2	14
#define NPCM8XX_CLK_APB1	15
#define NPCM8XX_CLK_APB5	16
#define NPCM8XX_CLK_CLKOUT	17
#define NPCM8XX_CLK_GFX		18
#define NPCM8XX_CLK_SU		19
#define NPCM8XX_CLK_SU48	20
#define NPCM8XX_CLK_SDHC	21
#define NPCM8XX_CLK_SPI0	22
#define NPCM8XX_CLK_SPI1	23
#define NPCM8XX_CLK_SPIX	24
#define NPCM8XX_CLK_RG		25
#define NPCM8XX_CLK_RCP		26
#define NPCM8XX_CLK_PRE_ADC	27
#define NPCM8XX_CLK_ATB		28
#define NPCM8XX_CLK_PRE_CLK	29
#define NPCM8XX_CLK_TH		30
	    
#define NPCM8XX_CLK_REFCLK	31
#define NPCM8XX_CLK_SYSBYPCK	32
#define NPCM8XX_CLK_MCBYPCK	33
	    
#define NPCM8XX_NUM_CLOCKS	(NPCM8XX_CLK_MCBYPCK+1)

#endif
