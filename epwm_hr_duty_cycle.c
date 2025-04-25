#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

//Frequency of PWM output signal in Hz - 1 KHz is selected
#define APP_EPWM_OUTPUT_FREQ    (1U * 1000U)
//APP run time in seconds
#define APP_EPWM_RUN_TIME    (60U)
//To be removed after syscfg integration
#define APP_INT_IS_PULSE    (1U)

//Global variables and objects
static HwiP_Object  gEpwmHwiObject;
static SemaphoreP_Object  gEpwmSyncSemObject;

//Function prototypes
static void App_epwmIntrISR(void *handle);

//Variables to hold base addresses of EPWM that are used
uint32_t gEpwmBaseAddr;
uint32_t gEpwmBaseAddr1;
uint32_t gEpwmBaseAddr2;
uint32_t gEpwmBaseAddr3;
uint32_t gEpwmBaseAddr4;
uint32_t gEpwmBaseAddr5;

///////////////////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <math.h>

//Initialization of arrays for plotting
float t;
float dt;

float CMPRone[120] = {0};
float CMPRtwo[120] = {0};
float CMPRthree[120] = {0};
float CMPRfour[120] = {0};
float CMPRfive[120] = {0};
float CMPRsix[120] = {0};
float napon_alfa[120] = {0};
float napon_beta[120] = {0};
float sektor_polje[120] = {0};
float polje_izlaz1[120] = {0};
float izlaz=0;


//DutyCycles structure
typedef struct {
    float d1;
    float d2;
    float d3;
} DutyCycles;

float Vaa;
float Vbb;
float Vcc;
int subsektor;
int sektor;

float alpha, beta;

//CMPR structure
typedef struct {
    float CMPR1;
    float CMPR2;
    float CMPR3;
    float CMPR4;
    float CMPR5;
    float CMPR6;
} CMPRValues;


//2-level SVPWM function
DutyCycles Svgen_dq_2_Level_modified(float alpha_mapped, float beta_mapped, int MainSector, float Vdc) {
    DutyCycles dc;

    alpha_mapped=3*alpha_mapped/(sqrt(3)*Vdc);
    beta_mapped=3*beta_mapped/(sqrt(3)*Vdc);

    float tmp1 = beta_mapped;
    float tmp2 = ((beta_mapped / 2) + (0.866025 * alpha_mapped));
    float tmp3 = tmp2 - tmp1;

    int VecSector = 3;
    VecSector = (tmp2 > 0) ? (VecSector - 1) : VecSector;
    VecSector = (tmp3 > 0) ? (VecSector - 1) : VecSector;
    VecSector = (tmp1 < 0) ? (7 - VecSector) : VecSector;

    if (VecSector == 1 || VecSector == 4) {
        dc.d1 = tmp2;
        dc.d2 = tmp1 - tmp3;
        dc.d3 = -tmp2;
    }
    else if (VecSector == 2 || VecSector == 5) {
        dc.d1 = tmp3 + tmp2;
        dc.d2 = tmp1;
        dc.d3 = -tmp1;
    }
    else {
        dc.d1 = tmp3;
        dc.d2 = -tmp3;
        dc.d3 = -(tmp1 + tmp2);
    }
    subsektor = VecSector;
    return dc;
}

//Function for duty assign
CMPRValues DutyAssign(DutyCycles dc, int main_sector) {
    CMPRValues cmpr;

    switch (main_sector) {
    case 1:
           cmpr.CMPR1 = dc.d1 + 0.5;
           cmpr.CMPR2 = 0;
           cmpr.CMPR3 = 0;
           cmpr.CMPR4 = dc.d2 - 0.5;
           cmpr.CMPR5 = 0;
           cmpr.CMPR6 = dc.d3 - 0.5;
           break;
       case 2:
           cmpr.CMPR1 = dc.d1 + 0.5;
           cmpr.CMPR2 = 0;
           cmpr.CMPR3 = dc.d2 + 0.5;
           cmpr.CMPR4 = 0;
           cmpr.CMPR5 = 0;
           cmpr.CMPR6 = dc.d3 - 0.5;
           break;
       case 3:
           cmpr.CMPR1 = 0;
           cmpr.CMPR2 = dc.d1 - 0.5;
           cmpr.CMPR3 = dc.d2 + 0.5;
           cmpr.CMPR4 = 0;
           cmpr.CMPR5 = 0;
           cmpr.CMPR6 = dc.d3 - 0.5;
           break;
       case 4:
           cmpr.CMPR1 = 0;
           cmpr.CMPR2 = dc.d1 - 0.5;
           cmpr.CMPR3 = dc.d2 + 0.5;
           cmpr.CMPR4 = 0;
           cmpr.CMPR5 = dc.d3 + 0.5;
           cmpr.CMPR6 = 0;
           break;
       case 5:
           cmpr.CMPR1 = 0;
           cmpr.CMPR2 = dc.d1 - 0.5;
           cmpr.CMPR3 = 0;
           cmpr.CMPR4 = dc.d2 - 0.5;
           cmpr.CMPR5 = dc.d3 + 0.5;
           cmpr.CMPR6 = 0;
           break;
       case 6:
           cmpr.CMPR1 = dc.d1 + 0.5;
           cmpr.CMPR2 = 0;
           cmpr.CMPR3 = 0;
           cmpr.CMPR4 = dc.d2 - 0.5;
           cmpr.CMPR5 = dc.d3 + 0.5;
           cmpr.CMPR6 = 0;
           break;
       default:
           cmpr.CMPR1 = 0;
           cmpr.CMPR2 = 0;
           cmpr.CMPR3 = 0;
           cmpr.CMPR4 = 0;
           cmpr.CMPR5 = 0;
           cmpr.CMPR6 = 0;
           break;
       }
    return cmpr;
}

//Main sector calculation function
int MainSectorCal(float Va, float Vb, float Vc) {
    if (Va > 0 && Vb < 0 && Vc < 0) {
        return 1;
    }
    else if (Va > 0 && Vb > 0 && Vc < 0) {
        return 2;
    }
    else if (Va < 0 && Vb > 0 && Vc < 0) {
        return 3;
    }
    else if (Va < 0 && Vb > 0 && Vc > 0) {
        return 4;
    }
    else if (Va < 0 && Vb < 0 && Vc > 0) {
        return 5;
    }
    else if (Va > 0 && Vb < 0 && Vc > 0) {
        return 6;
    }
    else {
        return -1; // Error case
    }
}

//Inverse Clarke transform
void InvClarkeConv(float Alpha, float Beta, float* Va, float* Vb, float* Vc) {
    *Va = Alpha;
    *Vb = -0.5 * Alpha + (sqrt(3) / 2) * Beta;
    *Vc = -0.5 * Alpha - (sqrt(3) / 2) * Beta;
}

//Alpha-Beta voltages
typedef struct {
    float Alpha;
    float Beta;
} AlphaBeta;

//Vector mapping function
AlphaBeta MapVector(float Alpha, float Beta, int MainSector, float Vdc) {
    AlphaBeta ab;
    switch (MainSector) {
    case 1:
        ab.Alpha = Alpha - (Vdc / 3);
        ab.Beta = Beta;
        break;
    case 2:
        ab.Alpha = Alpha - (Vdc / 6);
        ab.Beta = Beta - (sqrt(3) * Vdc / 6);
        break;
    case 3:
        ab.Alpha = Alpha + (Vdc / 6);
        ab.Beta = Beta - (sqrt(3) * Vdc / 6);
        break;
    case 4:
        ab.Alpha = Alpha + (Vdc / 3);
        ab.Beta = Beta;
        break;
    case 5:
        ab.Alpha = Alpha + (Vdc / 6);
        ab.Beta = Beta + (sqrt(3) * Vdc / 6);
        break;
    case 6:
        ab.Alpha = Alpha - (Vdc / 6);
        ab.Beta = Beta + (sqrt(3) * Vdc / 6);
        break;
    default:
        ab.Alpha = Alpha;
        ab.Beta = Beta;
        break;
    }
    return ab;
}

//MAIN FUNCTION
CMPRValues Svgen_dq_3_Level(float alpha, float beta) {
    DutyCycles dc;

    //Step 1: Inverse Clarke transform
    float Va, Vb, Vc;
    InvClarkeConv(alpha, beta, &Va, &Vb, &Vc);

    //Step 2: Main sector calculation
    int main_sector = MainSectorCal(Va, Vb, Vc);

    //Step 3: Vector mapping
    float Vdc = 700;
    AlphaBeta mapped = MapVector(alpha, beta, main_sector, Vdc);

    //Step 4: 2-level SVPWM
    dc = Svgen_dq_2_Level_modified(mapped.Alpha, mapped.Beta,main_sector, Vdc);

    //Step 5: Duty cycle assignment
    CMPRValues cmpr = DutyAssign(dc, main_sector);

    Vaa = Va;
    Vbb = Vb;
    Vcc = Vc;
    sektor = main_sector;

    return cmpr;
}




void epwm_hr_duty_cycle_main(void *args)
{
    t = 0.0002;
    dt = 0.001;
    ///////////////////////////////////////////////////////////////////////////////////////////
    int32_t  status;
    uint32_t  numIsrCnt = (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ);
    HwiP_Params  hwiPrms;

    //Open drivers to open the UART driver for console
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM Duty Cycle Test Started ...\r\n");
    DebugP_log("App will wait for 60 seconds (using PWM period ISR) ...\r\n");

    //Get address of ePWM
    gEpwmBaseAddr = CONFIG_EPWM1_BASE_ADDR;
    gEpwmBaseAddr1 = CONFIG_EPWM2_BASE_ADDR;
    gEpwmBaseAddr2 = CONFIG_EPWM3_BASE_ADDR;
    gEpwmBaseAddr3 = CONFIG_EPWM4_BASE_ADDR;
    gEpwmBaseAddr4 = CONFIG_EPWM5_BASE_ADDR;
    gEpwmBaseAddr5 = CONFIG_EPWM6_BASE_ADDR;

    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    //Register & enable interrupt
    HwiP_Params_init(&hwiPrms);
    //Integrate with Syscfg
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_epwmIntrISR;
    // Integrate with Syscfg
    hwiPrms.isPulse     = APP_INT_IS_PULSE;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr1);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr2);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr3);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr4);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr5);


    while(numIsrCnt > 0)
    {
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                alpha = 280*sqrt(2)*sin(2 * 3.14159 * 50 * (t));
                beta = 280*sqrt(2)*sin(2 * 3.14159 * 50 * (t) -3.14159/2);

                //Main 3-level SVPWM function
                CMPRValues cmpr = Svgen_dq_3_Level(alpha, beta);

                //Printing values in each execution cycle (optional)
                /*printf("t = %f\n", t);
                printf("CMPR1: %f\n", cmpr.CMPR1);
                printf("CMPR2: %f\n", cmpr.CMPR2);
                printf("CMPR3: %f\n", cmpr.CMPR3);
                printf("CMPR4: %f\n", cmpr.CMPR4);
                printf("CMPR5: %f\n", cmpr.CMPR5);
                printf("CMPR6: %f\n", cmpr.CMPR6);
                printf("Sektor: %d\n", sektor);
                printf("Subsektor: %d\n", subsektor);
                printf("Va: %f\n", Vaa);
                printf("Vb: %f\n", Vbb);
                printf("Vc: %f\n", Vcc);
                printf("Alfa: %f\n", alpha);
                printf("Beta: %f\n", beta);*/

                EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 25000-25000*cmpr.CMPR1);
                EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 25000+25000*cmpr.CMPR2);
                EPWM_setCounterCompareValue(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 25000-25000*cmpr.CMPR3);
                EPWM_setCounterCompareValue(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 25000+25000*cmpr.CMPR4);
                EPWM_setCounterCompareValue(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 25000-25000*cmpr.CMPR5);
                EPWM_setCounterCompareValue(CONFIG_EPWM6_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 25000+25000*cmpr.CMPR6);

        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);   //delay
        numIsrCnt--;
        t += dt;
    }

    EPWM_disableInterrupt(gEpwmBaseAddr);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);     //Clear any pending interrupts if any
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);

    DebugP_log("EPWM Duty Cycle Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_epwmIntrISR(void *handle)
{
    volatile bool status;

    status = EPWM_getEventTriggerInterruptStatus(gEpwmBaseAddr);
    if(status == true)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);
    }

    return;
}
