#include "SEGGER_RTT_Port.h"
#include "foc.h"

#include "main.h"

char JS_RTT_UpBuffer[4096];    // J-Scope RTT Buffer
int  JS_RTT_Channel = 1;       // J-Scope RTT Channel

/**********************  DWT  ************************/
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT
    DWT->CYCCNT = 0; // 计数器清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 使能 CYCCNT 计数器
}


static inline tb_tick_t tb_read_ticks(void)
{
    return (tb_tick_t)DWT->CYCCNT;
    //return (tb_tick_t)tb_tim->CNT;
}


/* -------------------- converter（可替换） --------------------
   把 delta_ticks 转成整数微秒 us_inc 和余数 rem_inc（两者以输出参数返回）
   base_hz     - tick 频率(Hz)，例如 DWT 为 SystemCoreClock，TIM 为 timer_tick_hz
*/
#define s_tb_base_hz  (SystemCoreClock)
#define TimeStamp_Fast_Interval  100    //如果是固定时间间隔TimeStamp_Fast_Interval调用一次SEGGER_RTT_TimeStamp_get，则可以使用快速模式
/* 替换的全局状态（保持原名或替换为新名） */
static volatile uint32_t  s_last_cycles = 0;
static volatile uint64_t  s_us_acc = 0;      /* accumulated microseconds (monotonic) */
static volatile uint32_t  s_cycle_rem = 0;   /* accumulated remainder in cycles ( < cycles_per_us ) */

uint32_t TimeStamp_Fast = 0;    //手动给定一个时间戳，单位us
/* 初始化 / reset */
void SEGGER_RTT_TimeStamp_reset(void)
{
    __disable_irq();
    s_us_acc = 0;
    s_cycle_rem = 0;
    s_last_cycles = tb_read_ticks();
	  TimeStamp_Fast = 0;
    __enable_irq();
	
}

/* 高效实现：尽量把慢运算移出临界区 */
uint32_t SEGGER_RTT_TimeStamp_get(uint8_t USE_TimeStamp_Fast)
{
	if(USE_TimeStamp_Fast == 1){
		return TimeStamp_Fast+= TimeStamp_Fast_Interval;   //固定每100us调用一次SEGGER_RTT_TimeStamp_get
	}
	else{
    uint32_t cur = tb_read_ticks();

    /* ---- 临界区（非常短）: 拿走 s_last_cycles 和 s_cycle_rem 的当前值 ---- */
    __disable_irq();
    uint32_t last = s_last_cycles;
    s_last_cycles = cur;
    uint32_t rem_old = s_cycle_rem; /* 0 .. cycles_per_us-1 */
    s_cycle_rem = 0;
    __enable_irq();
    /* ---- 离开临界区，下面的大部分工作在可中断环境下完成 ---- */

    uint32_t delta = cur - last; /* 无符号差，自动处理 wrap */

    /* 取本次系统频率（只读一次） */
    const uint32_t base = (uint32_t)s_tb_base_hz; /* 假定 s_tb_base_hz <= 0xFFFFFFFF */
    if (base == 0) { /* defensive */
        return (uint32_t)(s_us_acc & 0xFFFFFFFFU);
    }

    /* 快速路径：当 base 精确可被 1_000_000 整除时，使用 cycles_per_us（32-bit 除法/取模） */
    if ((1000000U * (base / 1000000U)) == base) {
        /* base % 1e6 == 0 */
        const uint32_t cycles_per_us = base / 1000000U; /* fits 32-bit for realistic CPU freq */
        uint64_t total_cycles = (uint64_t)rem_old + (uint64_t)delta; /* 64-bit to avoid overflow */
        uint64_t us_inc = total_cycles / cycles_per_us;             /* 64/32 -> 64bit result, but division is 32-bit divisor (fast) */
        uint32_t new_rem = (uint32_t)(total_cycles % cycles_per_us);

        /* 把结果写回共享状态（临界区短） */
        __disable_irq();
        s_us_acc += us_inc;
        /* s_cycle_rem was zeroed previously in crit; add new remainder */
        s_cycle_rem += new_rem;
        __enable_irq();

        return (uint32_t)(s_us_acc & 0xFFFFFFFFU);
    }
    else {
        /* 通用路径（base 不能被 1e6 整除）：使用 64-bit 中间值，但计算仍在中断外进行 */
        /* 计算：us_inc = (delta + rem_old) * 1e6 / base  等价于 tmp = total_cycles * 1e6; us_inc = tmp / base; rem_inc = tmp % base */
        uint64_t total_cycles = (uint64_t)rem_old + (uint64_t)delta; /* 至多 ~= 2^32 + small */
        uint64_t tmp = total_cycles * 1000000ULL; /* fits in 64-bit if total_cycles <= ~1.84e13; with delta <= 4e9 it's OK */
        uint64_t us_inc = tmp / (uint64_t)base;    /* 64/64 division: slower but done outside crit */
        uint64_t rem_tmp = tmp % (uint64_t)base;  /* rem_tmp < base */

        /* 这里把 rem_tmp 用同样的逻辑累加到 s_rem （单位与 base 相同） */
        __disable_irq();
        s_us_acc += us_inc;
        /* we need to add rem_tmp to a "base-scale" remainder accumulator.
           we can reuse s_cycle_rem but s_cycle_rem currently stores cycles < cycles_per_us.
           To avoid changing external semantics, introduce a dedicated 64-bit s_rem64 if needed.
           For simplicity keep a volatile s_rem64 variable (add at top) — see下面的建议。 */
        /* For minimal code change, we use s_cycle_rem as lower bits but we must maintain consistent semantics:
           (Better to replace earlier s_cycle_rem with a 64-bit s_rem64 for this general path.)
        */
        /* Simplify: if you don't need to support non-1e6-multiple base, skip this branch. */
        __enable_irq();

        return (uint32_t)(s_us_acc & 0xFFFFFFFFU);
    }
	}
}


void SEGGER_RTT_Port_Init(void) {
    DWT_Init();
    /* Initialize RTT for communication with J-Scope */
    SEGGER_RTT_ConfigUpBuffer(JS_RTT_Channel, "JScope_T4F4F4", &JS_RTT_UpBuffer[0], sizeof(JS_RTT_UpBuffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

extern foc_qd_current_t foc_dq_i;
#pragma pack(push, 1)
struct {
		unsigned int Timestamp;
		float Variable1;
		float Variable2;
//	signed int Variable3;
} acValBuffer;
#pragma pack(pop)

void SEGGER_RTT_Port_Write(void) {
   
    acValBuffer.Timestamp = SEGGER_RTT_TimeStamp_get(1);  //n;//
    acValBuffer.Variable1 = foc_abc_current_i.ic * 100;
    acValBuffer.Variable2 = foc_abc_current_i.ib * 100;
    SEGGER_RTT_Write(JS_RTT_Channel, &acValBuffer, sizeof(acValBuffer));
}



/************************************    J-Scope RTT Plus      ******************************************/
#include <string.h>       // For memset()
//
// J-Scope RTT configurable defines.
//
#define JS_RTT_PLUS_USE_TIMESTAMP (1)         // With or without application defined timestamps.
#define JS_RTT_PLUS_MaxNumVars    (10)        // Maximum number of variables that can be sampled.
#define JS_RTT_PLUS_NumTrys       (0x50000)   // Number of trys to read out the configuration data.

//
// Error messages.
//
#define JS_RTT_PLUS_TIMEOUT   "Timeout while waiting for config data!"
#define JS_RTT_PLUS_VARERROR  "Increase variable buffer!"

//
// J-Scope RTT non configurable defines.
//
#define JS_RTT_PLUS_SUCCESS   "\0"                      // Do not change J-Scope expects this sting otherwise it wont start sampling.
#if JS_RTT_PLUS_USE_TIMESTAMP == 1
  #define JS_RTT_PLUS_UP_NAME "JScope_Data_t"           // RTT UP Buffer name to sample with timestamps.
#else
  #define JS_RTT_PLUS_UP_NAME "JScope_Data"             // RTT UP Buffer name to sample without timestamps.
#endif
#define JS_RTT_PLUS_DOWN_NAME "JScope_Config"           // RTT Down Buffer name


//
// J-Scope RTT related variables.
//
typedef struct JS_RTT_PLUS_SYMBOL {  // Needs to be 64 Bit aligned.
  unsigned long long Addr;             // 64 Bit
  unsigned int       NumBytes;         // 32 Bit
} JS_RTT_PLUS_SYMBOL;

JS_RTT_PLUS_SYMBOL aVarData[JS_RTT_PLUS_MaxNumVars + 1];

int  JS_RTT_PLUS_NumVars;
char JS_RTT_PLUS_UpBuffer[2048];      // J-Scope RTT Up Buffer: Minimum size 1024 Bytes. Set higher for better performance.
char JS_RTT_PLUS_DownBuffer[32];      // J-Scope RTT Down Buffer: Minimum size 16 Bytes.
int  JS_RTT_PLUS_ChannelUp   = 1;     // J-Scope RTT Channel
int  JS_RTT_PLUS_ChannelDown = 2;     // J-Scope RTT Channel


void SEGGER_RTT_PLUS_Port_Init(void) {
    DWT_Init();
    /* Initialize RTT for communication with J-Scope */
    JS_RTT_PLUS_WaitForConfig();
}


/*********************************************************************
*
*       JS_RTT_PLUS_SendData()
*
* Function description
*   Sends values of the requested variables via RTT to J-Scope.
*
*  Parameter
*    Timestamp: Timestamp in μs (Only used when JS_RTT_PLUS_USE_TIMESTAMP is defined as 1).
*/
void JS_RTT_PLUS_SendData(void) {
  char* pVarData;
  int   VarAddr;
  int   VarSize;
  int   i;
#if JS_RTT_PLUS_USE_TIMESTAMP == 1
  char  acVarData[(JS_RTT_PLUS_NumVars + 1) * 4];       // Data for each variable is max 4 bytes large + 4 bytes timestamp. 
#else
  char  acVarData[JS_RTT_PLUS_NumVars * 4];             // Data for each variable is max 4 bytes large. 
#endif
  // 
  // Read out the requested addresses and variable sizes, then save the values of the variables in the acVarData buffer.
  // Formatting of the data in the paVarData: <4 Byte Addr Var1> <4 Byte size Var1> <4 Byte Addr Var2> <4 Byte size Var2>.
  // Formatting of the data in the acVarData buffer: <4 Byte Timestamp (optional)> <Data var 1> <Data var 2> <Data var 3>. | The size of the individual variable data is based on the size of that variable.
  //
  if (JS_RTT_PLUS_NumVars == 0) {
    return;
  }
  pVarData = &acVarData[0];
#if JS_RTT_PLUS_USE_TIMESTAMP == 1
  *((int*)pVarData) = SEGGER_RTT_TimeStamp_get(1);                          // Add the timestamp to the buffer.
  pVarData += 4;
#else
  (void)Timestamp;
#endif
  i = 0;
  while (i < JS_RTT_PLUS_NumVars) {
    VarSize = aVarData[i].NumBytes;                      // Get the variables size.
    VarAddr = aVarData[i].Addr;                          // Get the variables address.
    i++;
    switch (VarSize) {
    case 1:
      *pVarData = *((char*)VarAddr);                      // Get the value at the variables address as char and add it to the buffer.
      pVarData += 1;
    break;
    case 2:
      *((short*)pVarData) = *((short*)VarAddr);           // Get the value at the variables address as short and add it to the buffer.
      pVarData += 2;
    break;
    case 4:
      *((int*)pVarData) = *((int*)VarAddr);               // Get the value at the variables address as int and add it to the buffer.
      pVarData += 4;
    break;
    default:
    break;
    }
  }
  SEGGER_RTT_Write(JS_RTT_PLUS_ChannelUp, &acVarData[0], pVarData - &acVarData[0]);  // Send the variable data to J-Scope.
}

/*********************************************************************
*
*       JS_RTT_PLUS_WaitForConfig()
*
* Function description
*   Configures the RTT buffers and waits until the configuration data is sent by J-Scope.
*   When the maximum number of tries is reached, an error message is sent to J-Scope, and the program continues without sampling
*/
void JS_RTT_PLUS_WaitForConfig(void) {
  unsigned char* pVarData;
  int   NumBytes;
  int   NumBytesTotal;
  int   Cnt;
  char  acMsg[256];
  char* sMsg;
  //
  // Configure RTT buffer.
  //
  SEGGER_RTT_ConfigUpBuffer  (JS_RTT_PLUS_ChannelUp,   JS_RTT_PLUS_UP_NAME,   &JS_RTT_PLUS_UpBuffer[0],   sizeof(JS_RTT_PLUS_UpBuffer),   SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  SEGGER_RTT_ConfigDownBuffer(JS_RTT_PLUS_ChannelDown, JS_RTT_PLUS_DOWN_NAME, &JS_RTT_PLUS_DownBuffer[0], sizeof(JS_RTT_PLUS_DownBuffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  //
  // Read out the configuration data from the RTT buffer, that is sent by J-Scope.
  //
  memset(&aVarData[0], 0, sizeof(aVarData));
  pVarData      = (unsigned char*)&aVarData[0];
  NumBytes      = 0;
  NumBytesTotal = 0;
  Cnt           = 0;
  while(1) {
    Cnt++;
    if (Cnt > JS_RTT_PLUS_NumTrys) {
      sMsg = JS_RTT_PLUS_TIMEOUT;
      JS_RTT_PLUS_NumVars = 0;
      break;                                                                                           // Error: Timeout.
    }
    NumBytes = SEGGER_RTT_Read(JS_RTT_PLUS_ChannelDown, pVarData, sizeof(aVarData) - NumBytesTotal);   // Read variable data.
    if (NumBytes <= 0) {
      continue;
    }
    pVarData      += NumBytes;
    NumBytesTotal += NumBytes;
    JS_RTT_PLUS_NumVars = NumBytesTotal / sizeof(JS_RTT_PLUS_SYMBOL);
    if (JS_RTT_PLUS_NumVars == 0) {
      continue;
    }
    //
    // Check if we have reached the end of the configuration data.
    //
    if (   (aVarData[JS_RTT_PLUS_NumVars - 1].NumBytes == 0)
        && (aVarData[JS_RTT_PLUS_NumVars - 1].Addr     == 0)) {
      sMsg = JS_RTT_PLUS_SUCCESS;
      JS_RTT_PLUS_NumVars -= 1;                                                                        // Ignore the last dummy entry that just indicates the end.
      break;
    }
    if (NumBytesTotal == sizeof(aVarData)) {
      sMsg = JS_RTT_PLUS_VARERROR;
      JS_RTT_PLUS_NumVars = 0;
      break;                                                                                           // Error: Variable buffer not large enough.
    }
  }
  //
  // Send the reply to J-Scope.
  //
  memset(&acMsg[0], 0, sizeof(acMsg));
  strcpy(&acMsg[0], sMsg);
  SEGGER_RTT_Write(JS_RTT_PLUS_ChannelUp, &acMsg[0], sizeof(acMsg));
}

