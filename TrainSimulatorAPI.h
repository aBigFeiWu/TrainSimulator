#ifndef TRAINSIMULATOR_API_H
#define TRAINSIMULATOR_API_H

// 根据是否在构建DLL来决定是导出还是导入符号
#ifdef _WIN32
    #ifdef TRAINSIMULATOR_EXPORTS
        #define API_DECL __declspec(dllexport)
    #else
        #define API_DECL __declspec(dllimport)
    #endif
#else
    #define API_DECL
#endif

// C# 可以直接映射的简单结构体
struct KinematicState_C {
    double position;
    double velocity;
    double acceleration;
    double jerk;
};

// 新增：用于API层的大地坐标结构体
struct GeodeticPoint_C {
    double lon; // 经度
    double lat; // 纬度
    double alt; // 高程
};

struct SimulatorConfig_C {

    //指令相关,默认都赋值=1即可
    int trajectory_ID = 1;
    int trajectory_type = 1;
    int trajectory_ID_user2 = 2;
    int trajectory_type_user2 = 1;
    int enable_second_user = 0; // 0: 单用户；1: 双用户

    //TestSequence
    const wchar_t* SequenceID;

    //SimulateConfiguration
    const wchar_t* EphemerisFile;        //星历配置
    const wchar_t* ConstellationConfigration; //星座配置
    int simulation_interval_ms;
    const wchar_t* ip;
    int port;
    const wchar_t* SimTime;   //时间配置
    long long simulation_start_time;
    long long simulation_duration;
    int SimMode;

    //TestTrack
    const wchar_t* TrackID;
    const wchar_t* StartStation;
    const wchar_t* StartTrack;
    const wchar_t* EndStation;
    const wchar_t* EndTrack;
    const wchar_t* TrackFlie;
    const wchar_t* route_file_path;//route_file_path就是TrackFlie

    //TestVehicle
    const wchar_t* TrainType;
    double MaxSpeed;//"MaxSpeed(km/h)"
    double TrainLong;//"TrainLong(m)"
    double Resistance[3];//"Resistance": "2.28 \u002B 0.0293v \u002B 0.000178v²" ,     //基本阻力
    double TractionAcceleration;    //"TractionAcceleration(m/s^2)": 0.59,  //最大加速度
    double BrakeAcceleration;    //"BrakingAcceleration(m/s^2)": -0.89,  //最大减速度

    //TestCases
    const wchar_t* CaseId; //": "CN-TI-1",
    const wchar_t* CaseName;//": "北斗B1Ⅰ频点信号接收",
    const wchar_t* TestContent;//": "将板卡连接到卫星信号模拟器，设置模拟器输出北斗B1Ⅰ频点信号，给板卡上电，通过板卡配置工具查看板卡，成功接收到北斗B1Ⅰ频点的信号。"

};

// extern "C" 防止 C++ 编译器进行名称修饰 (name mangling)
// 使得 C# 可以通过函数名找到它们
#ifdef __cplusplus
extern "C" {
#endif

    // 初始化与销毁
    API_DECL void* CreateSimulator(SimulatorConfig_C config);
    API_DECL void DestroySimulator(void* simulator_handle);

    // 仿真控制
    API_DECL bool StartSimulation(void* simulator_handle);
    API_DECL bool StopSimulation(void* simulator_handle);

    // 模式控制
    API_DECL void SetControlMode(void* simulator_handle, int mode); // 0: Auto, 1: Manual
    API_DECL void SetControlLevel(void* simulator_handle, int level); // 对应 TrainController::ControlLevel 枚举

    // 数据获取
    API_DECL KinematicState_C GetCurrentState(void* simulator_handle);

    // --- 新增的API函数 ---
    // 获取当前速度 (m/s)
    API_DECL double GetCurrentSpeed(void* simulator_handle);

    // 获取当前大地坐标 (经纬高)
    API_DECL GeodeticPoint_C GetCurrentPositionBLH(void* simulator_handle);


#ifdef __cplusplus
}
#endif

#endif //TRAINSIMULATOR_API_H
