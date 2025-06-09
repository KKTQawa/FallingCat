//#define WITHOUT_NUMPY

#include"matplotlibcpp.h"

namespace plt = matplotlibcpp;
#ifdef B3_USE_ROBOTSIM_GUI
#include "b3RobotSimulatorClientAPI.h"
#else
#include "b3RobotSimulatorClientAPI_NoGUI.h"
#endif

#include "../Utils/b3Clock.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>
#define ASSERT_EQ(a, b) assert((a) == (b));
#include "MinitaurSetup.h"
#include "btBulletDynamicsCommon.h"
#include "Bullet3Common/b3Quaternion.h"
#include <LinearMath/btConvexHull.h>
#include <LinearMath/btConvexHullComputer.h>
#include  <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include<SharedMemoryPublic.h>

#include<iostream>
#include<unordered_map>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include<fstream>
#include<filesystem>
namespace fs = std::filesystem;
#include "FastNoiseLite.h"//用于自然随机算法
PyObject* to_python(const std::vector<double>& vec) {
    return plt::detail::get_array(vec); // matplotlibcpp 内置转换函数
}

struct SharedData {
    std::mutex mtx;//互斥锁
    std::condition_variable cv;//线程通知者
    //时间，高度，线/角速度，阻力，估计空气阻力，xy平面投影面积，xyz轴和旋转方向的转动惯量,xy质心
    std::deque<std::tuple<double, double, double, double,double,double,double,double, double, double, double, double, double>> data_deque;  //共享数据
    std::atomic<bool> sim_running{true};//线程运行变量
    size_t max_data_points = 10000;  // 限制最大数据点数
    double x_right = 30.0;
    double delta_time = 30.0;
};
struct camera {
    
    double yaw ;//水平旋转
    double pitch ;//上下旋转，正值表示上仰
    double roll ;
    double distance ;//用于控制前进后退
    double offset ;//用于控制升降
    camera(double y = 208, double p = 346, double r = 0, double d = 3, double o = 0.5)
        : yaw(y), pitch(p), roll(r), distance(d), offset(o) {
    }
};
struct Physics_SIM
{    
    std::unordered_map<std::string, int> model_ids_;
    b3RobotSimulatorClientAPI* sim_= nullptr;
    btScalar fixedTimeStep = 1. / 240.;
    btQuaternion init_ori= btQuaternion(0,0.706825,0.707388,0);
    btQuaternion init_orient;
    int numJoints = 0;
    double Mass = 0.0;
    //调试栏相关
    std::unordered_map <std::string, int > params;
    std::vector<int>jointId;
    std::vector<std::pair<double, double>>limits;;

    double RESET_HEIGHT = 100.0; //重置高度
    double GROUND_MIN = 5.0;//接近地面的阈值
    double Max_Velocity = 500.0;  // 速度阈值，当速度接近地面时重置

    camera* cam = nullptr;
    //进程相关
    b3Clock clock;//累计暂停时长
    bool is_paused = true;
    bool is_auto = false;         //是否进行随机动画
    bool Allow_reset = true; //落地后是否重置高度
    bool is_graph = false;//是否开启绘图
    int cnt = 1;//记录下落次数

    double accumulated_paused_time = 0.0;  // 单次下落累计暂停时间（秒）
    double accumulated_paused_time2 = 0.0;// 总计累计暂停时间（秒）
    double start_time = 0.0;         // 暂停时刻的时钟值
    double start_time2 = 0.0; 
    double pause_start_time = 0.0;  //每次暂停开始时间

    int stop_t = 0;//落地停留时长

    //投影面积相关
    double min_Sxy = 0.05;
    double max_Sxy = 0.2;
    double Sxy = 0.1;
    
    btVector3 com;//质心位置
    btMatrix3x3 inertia_com;//惯性张量
    struct state {
        btVector3 pos;
        btQuaternion orient;//四元组朝向
        btVector3 v;
        btVector3 w;//角速度
    };
    struct shape {
        btVector3 localMin;
        btVector3 localMax;
        shape(btVector3 mi, btVector3 ma) :localMin(mi), localMax(ma) {}
    };
    state tmp;
    std::vector<shape>sh;
   //状态相关
    btVector3 drag=btVector3(0.0, 0.0, 0.0);//估计空气阻力
    double drag2 = 0.0;//实际z轴阻力

    btVector3 total_torque= btVector3(0, 0, 0);    // 总力矩（矢量）
    double total_force_magnitude=0.0; // 总冲击力大小（标量）

    //随机噪声对象
    FastNoiseLite noise;
    float target_valx = 1.0;
    float target_valy = 2.0;
    float target_valz = 3.0;
    //猫的体型
    double cat_height = 0.0;
    double cat_long = 0.0;
    double cat_width = 0.0;
    Physics_SIM()
    {
        sim_ = new b3RobotSimulatorClientAPI();
          cam = new camera();
        noise.SetNoiseType(FastNoiseLite::NoiseType_Perlin);  // 设置为 Perlin 噪声
        noise.SetFrequency(0.1f);                            // 设置噪声频率
    };
    ~Physics_SIM()
    {
        if (cam != nullptr)delete cam, cam = NULL;
        if (sim_->isConnected()){
            sim_->disconnect();
            std::cout << "delete sim" << std::endl;
            delete sim_;
            sim_ = nullptr;
        }
    }
    std::unordered_map<std::string, int> loadURDF()
    {
        std::unordered_map<std::string, int> model_ids;

        // 加载平面
            int planeId = sim_->loadURDF("plane.urdf");
            if ( planeId >= 0)
            {
                model_ids["plane"] = planeId;
                std::cout << "Successfully loaded plane.urdf (ID: " << planeId << ")\n";
            }
            else
            {
                std::cerr << "Error: Failed to load plane.urdf\n";
            }

            // 初始位姿
            b3RobotSimulatorLoadUrdfFileArgs args;
            args.m_startOrientation.setEulerZYX(3.14159, 0.0, 1.57);  // 绕 Y 轴旋转 90 度（π/2 弧度）
            args.m_startPosition.setValue(0, 0,100);
            // 加载URDF
            int robotId = sim_->loadURDF("laikago/laikago.urdf",args);
            btQuaternion q = args.m_startOrientation;

        if ( robotId >= 0)
        {
            model_ids["cat"] = robotId;
            std::cout << "Successfully loaded laikago.urdf (ID: " << robotId << ")\n";
        }
        else
        {
            std::cerr << "Error: Failed to load laikago.urdf\n";
        }

        return model_ids;
    }
    
    void window_init()
    {
        if (!sim_->isConnected())
            sim_->connect(eCONNECT_GUI);
        //Can also use eCONNECT_DIRECT,eCONNECT_SHARED_MEMORY,eCONNECT_UDP,eCONNECT_TCP, for example:
        //sim->connect(eCONNECT_UDP, "localhost", 1234);
        //sim_->configureDebugVisualizer(COV_ENABLE_GUI, 0);
        //	sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);//COV_ENABLE_WIREFRAME
        sim_->setTimeOut(10);

        sim_->setTimeStep(fixedTimeStep);
        sim_->setGravity(btVector3(0, 0, 0));
        sim_->setRealTimeSimulation(false);

        std::cout << "请输入落地停留时长（单位ms):";
        std::cin >> stop_t;
        model_ids_ = loadURDF();
         //syncBodies is only needed when connecting to an existing physics server that has already some bodies
        //sim_->syncBodies();
        for (auto it : model_ids_)
        {
            if (it.second < 0)
                std::cout << "Error loaded!" << it.first << std::endl;
            else
                std::cout << it.first << " " << it.second << std::endl;
        }
       
        int catId = model_ids_["cat"];
        //计算猫的高度
        btVector3 aabbMin, aabbMax;
        sim_->getAABB(catId, -1, aabbMin, aabbMax);
        cat_height =abs( aabbMax.z() - aabbMin.z());
        std::cout << "猫高为" << cat_height << std::endl;
        std::cout << "猫体长为" <<aabbMax.y() - aabbMin.y()  << std::endl;
        std::cout << "猫体宽为" <<aabbMax.x() - aabbMin.x()  << std::endl;
        cat_long = aabbMax.y() - aabbMin.y();
        cat_width = aabbMax.x() - aabbMin.x();
        // 获取关节数量
        numJoints = sim_->getNumJoints(catId);

        // 设置黄色
        btVector4 yellow(1.0, 1.0, 0.0, 1.0);
        
        for (int i = -1; i < numJoints; i++)
        {
            // 处理颜色信息
            b3RobotSimulatorChangeVisualShapeArgs cargs;
            cargs.m_objectUniqueId = catId; // 物体ID
            cargs.m_linkIndex = i;          // -1表示基座，其他表示关节索引
            cargs.m_hasRgbaColor = true;    // 启用颜色修改
            cargs.m_rgbaColor = yellow;     // 设置黄色

            sim_->changeVisualShape(cargs);

            b3JointInfo info;

            sim_->getJointInfo(catId, i, &info); // 获取关节信息
            // std::cout << info.m_jointName << " " << info.m_jointType << " " << info.m_jointIndex << std::endl;
            b3VisualShapeInformation ShapeImf;
            sim_->getVisualShapeData(catId, ShapeImf);
            b3VisualShapeData* shapes = ShapeImf.m_visualShapeData;
           //std::cout << shapes->m_visualGeometryType << std::endl;//5代表三角形网格
            // 填充AABB
            btVector3 localMin, localMax;
            sim_->getAABB(catId, i, localMin, localMax);
            sh.push_back(shape(localMin, localMax));
            // 如果是旋转或者平移关节,加入质量计算，改变物理世界初始状态
            if (info.m_jointType == 0 || info.m_jointType == 1)
            {
                double to_mass = 0;
                if (i == -1)
                    to_mass = 3;
                else if (i % 3 == 0)
                    to_mass = 0.3;
                else if (i % 3 == 1)
                    to_mass = 0.2;
                else if (i % 3 == 2)
                    to_mass = 0.1;
                if (i == 4 || i == 9)
                    to_mass = 0.4;
                double tt = 0.3;
                if (i == -1)tt = 0.8;
                double tk = 0.05;
                if (i != -1)tk = 0.008;
                b3RobotSimulatorChangeDynamicsArgs dargs;
                dargs.m_mass = to_mass;
                dargs.m_angularDamping =tt;
                dargs.m_linearDamping = tk;
                sim_->changeDynamics(
                    catId, i, dargs);
                Mass += to_mass; // 累加质量
                //std::cout << to_mass << "Mass:" << Mass << std::endl;
                if (i % 3 !=0&&i>0) {
                    sim_->setCollisionFilterGroupMask(catId,i , 1, 1);//开启下肢之间碰撞检测
                }
            }
            else {
                Mass += 3;//身体质量
                b3RobotSimulatorChangeDynamicsArgs dargs;
                dargs.m_mass = 3;
                dargs.m_angularDamping = 0.8;
                dargs.m_linearDamping = 0.1;
                sim_->changeDynamics(
                    catId, i, dargs);
            }
            
            // 给所有除了身体以外的关节添加调试参数
            if (i != -1)
            {
                b3JointSensorState State;
                sim_->getJointState(catId, i, &State);
                params[info.m_jointName] = sim_->addUserDebugParameter(info.m_jointName, info.m_jointLowerLimit, info.m_jointUpperLimit,
                    State.m_jointPosition);
                //保存关节相关的调试信息
                jointId.push_back(params[info.m_jointName]);
                limits.push_back({ info.m_jointLowerLimit, info.m_jointUpperLimit });
            }
        }
        // 添加额外调试参数
        params["Allow_Reset"] = sim_->addUserDebugParameter(
            "Allow_Reset", 0, 3, 3);
        params["Auto"] = sim_->addUserDebugParameter(
            "Auto", 0, 3, 0);
        params["draw"] = sim_->addUserDebugParameter(
            "draw", 0, 3, 0);
        params["RESET_HEIGHT"] = sim_->addUserDebugParameter(
            "RESET_HEIGHT", 50, 800,100);
        params["Max_Velocity"] = sim_->addUserDebugParameter(
            "Max_Velocity", 500, 2000, 500);
        params["Quit"] = sim_->addUserDebugParameter(
            "Quit", 1, 0, 0);
    }
};
//计算猫的质心
void compute_com(Physics_SIM& G, std::string name) {
    int body_id = G.model_ids_[name];
    btVector3 weighted_pos_sum(0.0,0.0,0.0);

    // 基座贡献
    b3DynamicsInfo base_info;
    if (G.sim_->getDynamicsInfo(body_id, -1, &base_info)) {
        if (base_info.m_mass > 0) {
            btVector3 base_pos;
            btQuaternion base_orn;
            G.sim_->getBasePositionAndOrientation(body_id, base_pos, base_orn);
            weighted_pos_sum += base_pos * base_info.m_mass;
        }
    }

    // 各链接贡献
    for (int link_idx = 0; link_idx < G.numJoints; link_idx++) {
        b3DynamicsInfo link_info;
        if (G.sim_->getDynamicsInfo(body_id, link_idx, &link_info)) {
            if (link_info.m_mass > 0) {
                b3LinkState link_state;
                if (G.sim_->getLinkState(body_id, link_idx,0,0,& link_state)){
                   // for (int x = 0;x < 3;x++)std::cout <<"hhh"<< link_state.m_worldPosition[x] << std::endl;
                    weighted_pos_sum +=  btVector3(link_state.m_worldPosition[0],link_state.m_worldPosition[1],link_state.m_worldPosition[2])* link_info.m_mass;
                }
            }
        }
    }
    G.com = weighted_pos_sum / G.Mass;
}
//计算物体围绕质心的惯性张量
void compute_inertia_com(Physics_SIM& G, std::string name) {
    int body_id = G.model_ids_[name];
    btMatrix3x3 total_inertia(0, 0, 0, 0, 0, 0, 0, 0, 0);
    btVector3 ref_point = G.com; // 使用之前计算的质心位置

    // 平行轴定理辅助函数
    auto add_inertia_contribution = [](const btVector3& pos,
        const btQuaternion& orn,
        const btVector3& local_inertia,
        double mass,
        const btVector3& ref) -> btMatrix3x3 {
            // 转换惯性张量到世界坐标系
            btMatrix3x3 rot_matrix(orn);
            btMatrix3x3 world_inertia = rot_matrix * btMatrix3x3(
                local_inertia.x(), 0, 0,
                0, local_inertia.y(), 0,
                0, 0, local_inertia.z()
            ) * rot_matrix.transpose();

            // 平行轴定理
            btVector3 r = pos - ref;
            btMatrix3x3 r_skew(
                0, -r.z(), r.y(),
                r.z(), 0, -r.x(),
                -r.y(), r.x(), 0
            );
            btScalar r_sq = r.length2();
            btMatrix3x3 parallel_term = r_skew * r_skew * mass;

            return world_inertia - parallel_term;
        };
  
    // 基座贡献
    b3DynamicsInfo base_info;
    if (G.sim_->getDynamicsInfo(body_id, -1, &base_info)) {
        if (base_info.m_mass > 0) {
            btVector3 base_pos;
            btQuaternion base_orn;
            G.sim_->getBasePositionAndOrientation(body_id, base_pos, base_orn);

            // 获取世界坐标系下的质心位置
            btVector3 world_com;
            btTransform base_transform(base_orn, base_pos);
            
            world_com = base_transform(btVector3(base_info.m_localInertialFrame[0],base_info.m_localInertialFrame[1],base_info.m_localInertialFrame[2]));

            btVector3 localInertiaVec(
                base_info.m_localInertialDiagonal[0],
                base_info.m_localInertialDiagonal[1],
                base_info.m_localInertialDiagonal[2]
            );

            total_inertia += add_inertia_contribution(
                world_com, base_orn, localInertiaVec,
                base_info.m_mass, ref_point);
        }
    }

    // 各链接贡献
    for (int link_idx = 0; link_idx < G.numJoints; link_idx++) {
        b3DynamicsInfo link_info;
        if (G.sim_->getDynamicsInfo(body_id, link_idx, &link_info)) {
            if (link_info.m_mass > 0) {
                b3LinkState link_state;
                if (G.sim_->getLinkState(body_id, link_idx,0,0 ,&link_state)) {
                    // 获取世界坐标系下的质心位置
                    btVector3 world_com;
                    //传入世界坐标下的位置和旋转状态，构建位姿对像
                        btTransform link_transform( btQuaternion(link_state.m_worldOrientation[0], link_state.m_worldOrientation[1], link_state.m_worldOrientation[2], link_state.m_worldOrientation[3] )
                            ,btVector3(link_state.m_worldPosition[0], link_state.m_worldPosition[1], link_state.m_worldPosition[2])
                        );
                       
                        //传入关节惯性系原点在连杆局部坐标系中的位置
                    world_com = link_transform( btVector3(link_info.m_localInertialFrame[0], link_info.m_localInertialFrame[1], 
                        link_info.m_localInertialFrame[2]));
                    btVector3 localInertiaDig(
                        base_info.m_localInertialDiagonal[0],
                        base_info.m_localInertialDiagonal[1],
                        base_info.m_localInertialDiagonal[2]
                    );
                    btQuaternion world_orn = btQuaternion(link_state.m_worldOrientation[0], link_state.m_worldOrientation[1],
                        link_state.m_worldOrientation[2], link_state.m_worldOrientation[3]);
                    total_inertia += add_inertia_contribution(
                        world_com, world_orn,localInertiaDig, link_info.m_mass, ref_point);}
            }
        }
    }

    G.inertia_com = total_inertia;
}
//根据凸包顶点集计算xy平面投影面积：Shoelace Formula（鞋带公式）
double computeProjectedAreaXY(btConvexHullShape* shape) {
    // 获取所有的顶点
    int numPoints = shape->getNumVertices();
    std::vector<btVector3> points;
    for (int i = 0; i < numPoints; ++i) {
        btVector3 vtx;
        shape->getVertex(i, vtx);
        points.push_back(vtx);
    }

    // 计算凸包在XY平面上的投影面积
    double area = 0.0;
    int j = points.size() - 1; // 最后一个点作为起始点
    for (unsigned int i = 0; i < points.size(); i++) {
        area += (points[j].x() + points[i].x()) * (points[j].y() - points[i].y());
        j = i;  // j is previous vertex to i
    }
    return fabs(area / 2.0);
}
//计算投影面积
void compute_Sxy(Physics_SIM& G,int Cat) {
    // 存储猫所有关节的所有顶点
    btAlignedObjectArray<btVector3> cat_vertices;
    // 使用aabb获取每个关节在x轴和y轴方向的最大最小位置
    for (int idx = -1; idx < G.numJoints; idx++) {
        b3CollisionShapeInformation shapeImf;
        if (G.sim_->getCollisionShapeData(Cat, idx, shapeImf)) {
            b3CollisionShapeData* shapeData = shapeImf.m_collisionShapeData;
            if (shapeData->m_collisionGeometryType != GEOM_SPHERE) {
                // 获取关节世界位姿
                b3LinkState linkState;
                G.sim_->getLinkState(Cat, idx, 1, 1, &linkState);

                // 构造关节变换矩阵
                btQuaternion orien(linkState.m_worldLinkFrameOrientation[0],
                    linkState.m_worldLinkFrameOrientation[1],
                    linkState.m_worldLinkFrameOrientation[2],
                    linkState.m_worldLinkFrameOrientation[3]);
                btVector3 pos(linkState.m_worldLinkFramePosition[0],
                    linkState.m_worldLinkFramePosition[1],
                    linkState.m_worldLinkFramePosition[2]);
                btTransform JointTransform(orien, pos);

                // 获取该关节的局部AABB
                btVector3 localMin = G.sh[idx + 1].localMin;
                btVector3 localMax = G.sh[idx + 1].localMax;

                // 生成AABB的8个角点
                btVector3 aabbCorners[8] = {
                    btVector3(localMin.x(), localMin.y(), localMin.z()),
                    btVector3(localMax.x(), localMin.y(), localMin.z()),
                    btVector3(localMin.x(), localMax.y(), localMin.z()),
                    btVector3(localMax.x(), localMax.y(), localMin.z()),
                    btVector3(localMin.x(), localMin.y(), localMax.z()),
                    btVector3(localMax.x(), localMin.y(), localMax.z()),
                    btVector3(localMin.x(), localMax.y(), localMax.z()),
                    btVector3(localMax.x(), localMax.y(), localMax.z())
                };

                // 变换到世界坐标系并收集顶点
                for (int i = 0; i < 8; ++i) {
                    btVector3 worldCorner = JointTransform * aabbCorners[i];
                    cat_vertices.push_back(worldCorner);
                }
            }
        }
    }

    // 计算投影面积
    if (cat_vertices.size() > 0) {
        // 正确构造方式：
        btConvexHullShape hullShape(
            reinterpret_cast<const btScalar*>(&cat_vertices[0]), // 转换为btScalar指针
            cat_vertices.size(),                                // 顶点数量
            sizeof(btVector3)                                   // 步长（默认值可省略）
        );
        G.Sxy = computeProjectedAreaXY(&hullShape);
    }
    G.min_Sxy = std::min(G.Sxy, G.min_Sxy);
    G.min_Sxy = std::max(G.Sxy, G.min_Sxy);
}
//估计空气阻力
void compute_aerodynamic_drag(Physics_SIM& G, int cat) {
    // 评估阻力系数Cd
    double Cd;
    if (G.max_Sxy > G.min_Sxy) {
        Cd = 0.3 + 0.9 * (G.Sxy - G.min_Sxy) / (G.max_Sxy - G.min_Sxy);
    }
    else {
        Cd = 0.3;
    }

    // 获取当前速度
    btVector3 base_velocity, base_angular;
        btVector3 base_position;
        btQuaternion base_orientation;

        G.sim_->getBasePositionAndOrientation(cat, base_position, base_orientation);
    G.sim_->getBaseVelocity(cat, base_velocity, base_angular);
    double base_velocity_magnitude = base_velocity.length();
    btVector3 drag;
if (base_velocity_magnitude > 1e-6) {
        // 计算阻力方向和大小
        btVector3 velocity_dir = base_velocity.normalized();

        double drag_magnitude = 0.5 * 1.225 * G.Sxy * Cd *
            base_velocity_magnitude * base_velocity_magnitude;
         drag = -drag_magnitude * velocity_dir;
         //drag[0] /= 1000.0;
         //drag[1] /= 1000.0;
         //G.sim_->applyExternalForce(cat, -1, drag, base_position,0);
}
else {
    drag = btVector3(0, 0, 0);
}
G.drag = drag;
}
//处理冲击力和力矩
void evaluate_impact_forces(Physics_SIM& G, int cat) {
    // 初始化累加变量
    btVector3 total_impulse_vec(0.0, 0.0, 0.0);  // 总冲量（矢量累加）
    btVector3 total_torque_vec(0.0, 0.0, 0.0);   // 总力矩（矢量累加）
    double total_force_mag = 0.0;                // 总冲击力大小（标量和）

    // 获取接触点信息
    b3RobotSimulatorGetContactPointsArgs contactArgs;
    contactArgs.m_bodyUniqueIdA = cat;
    contactArgs.m_bodyUniqueIdB = -1;
    b3ContactInformation contactInfo;
    if (!G.sim_->getContactPoints(contactArgs, &contactInfo)) {
        std::cerr << "获取接触点失败！" << std::endl;
        return;
    }

    if (contactInfo.m_numContactPoints > 0) {
        btVector3 base_pos;
        btQuaternion base_angular;
        G.sim_->getBasePositionAndOrientation(cat, base_pos, base_angular);

        for (int i = 0; i < contactInfo.m_numContactPoints; i++) {
            const b3ContactPointData& contact = contactInfo.m_contactPointData[i];

            // 1. 获取接触点位置和法线
            btVector3 contact_pos(
                contact.m_positionOnAInWS[0],
                contact.m_positionOnAInWS[1],
                contact.m_positionOnAInWS[2]
            );
            btVector3 normal(
                contact.m_contactNormalOnBInWS[0],
                contact.m_contactNormalOnBInWS[1],
                contact.m_contactNormalOnBInWS[2]
            );

            // 2. 计算当前接触点的力和力矩
            double normal_force = contact.m_normalForce;
            btVector3 force = normal * normal_force;       // 力矢量
            btVector3 r_vector = contact_pos - base_pos;   // 力臂

            // 3. 累加冲量、力矩和力大小
            //total_impulse_vec += force;                    // 矢量累加
            total_torque_vec += r_vector.cross(force);     // 力矩累加
            total_force_mag += force.length();             // 标量累加
        }
    }

    // 输出结果
    //std::cout << "总冲量大小: " << total_impulse_vec.length() << " N·s" << std::endl;
    //std::cout << "总力矩: ("
    //    << total_torque_vec.x() << ", "
    //    << total_torque_vec.y() << ", "
    //    << total_torque_vec.z() << ") N·m" << std::endl;
    //std::cout << "总冲击力大小: " << total_force_mag << " N" << std::endl;

    // 更新全局记录
    //if (total_impulse_vec.length() > G.total_impulse.length()) {
    //    G.total_impulse = total_impulse_vec;  // 更新为更大的冲量
    //}
    //用力*力矩来评估落地伤害
    if (total_force_mag * total_torque_vec.length() > G.total_force_magnitude * G.total_torque.length()) {
        G.total_force_magnitude = total_force_mag;
        G.total_torque = total_torque_vec;
    }
}
//处理键盘事件
void handleKeyboardEvents(Physics_SIM& G, int cat) {
    // 获取键盘事件
    b3KeyboardEventsData keyboardEvents;
    G.sim_->getKeyboardEvents(&keyboardEvents);

    // 处理持续按下的键
    for (int i = 0; i < keyboardEvents.m_numKeyboardEvents; i++) {
        const b3KeyboardEvent& e = keyboardEvents.m_keyboardEvents[i];
        int keyCode = e.m_keyCode;
        //处理单次按下的键
        if ((keyCode == ' ') && (e.m_keyState & eButtonTriggered)) {
            G.is_paused = !G.is_paused;
            if (G.is_paused) {
                G.pause_start_time= G.clock.getTimeInSeconds();
                // 保存状态
                G.sim_->getBasePositionAndOrientation(cat, G.tmp.pos, G.tmp.orient);
                G.sim_->getBaseVelocity(cat, G.tmp.v, G.tmp.w);

                // 设置速度为0，关闭重力
                G.sim_->setGravity(btVector3(0, 0, 0));
                G.sim_->resetBaseVelocity(cat, btVector3(0, 0, 0), btVector3(0, 0, 0));
            }
            else {
                double tmp_time= G.clock.getTimeInSeconds();
                G.accumulated_paused_time += tmp_time - G.pause_start_time;
                G.accumulated_paused_time2 += tmp_time - G.pause_start_time;
                // 恢复速度和重力
                G.sim_->resetBaseVelocity(cat, G.tmp.v, G.tmp.w);
                //G.sim_->resetBasePositionAndOrientation(cat, G.tmp.pos, G.tmp.orient);
                G.sim_->setGravity(btVector3(0, 0, -9.8));
            }
        }
        if ((keyCode == 'r') && (e.m_keyState & eButtonTriggered)) {
            G.cam->offset = 0.5;
            G.cam->distance = 3;
            G.cam->yaw = 208;
            G.cam->pitch = 346;
        }
        //处理持续按下的键,注意循环迭代很快
        if (e.m_keyState & eButtonIsDown) {
            switch (keyCode) {
            case 'd': G.cam->yaw += 0.2; break;
            case 'a': G.cam->yaw -= 0.2; break;
            case 'e': G.cam->pitch +=0.2; break;
            case 'q': G.cam->pitch -= 0.2; break;
            case 'f': G.cam->distance = std::min(G.cam->distance + 0.02, 10.0); break;
            case 'j': G.cam->distance = std::max(G.cam->distance - 0.02, 1.0); break;
            case 'b': G.cam->offset += 0.02; break;
            case 'n': G.cam->offset -= 0.02; break;
            }
        }
    }
}
//处理调试栏
bool controlJointsWithSliders(Physics_SIM& G, int cat,int frame ) {

    static double time_counter = 0.0;
    const double time_step = G.fixedTimeStep;

    std::vector<int>& jointIds = G.jointId;
    for (size_t i = 0; i < jointIds.size(); i++) {
        G.target_valx += 0.05;
        G.target_valy += 0.03;
        G.target_valz += 0.3;
        // 读取滑动条值
        double targetpos = G.sim_->readUserDebugParameter(jointIds[i]);

        if (G.is_auto&& ((i>6&&(frame%(i*10)==0))||(i<=6&&(frame%i*20==18))||(i==0&&frame%30==0))) {
            // 获取关节限制（limits已预先存储）
            double mi = G.limits[i].first;
            double ma =  G.limits[i].second;
            double leng = ma - mi;

            // 获得的值位于1到-1之间
            double random_value = G.noise.GetNoise(G.target_valx, G.target_valy, G.target_valz );
            //std::cout << random_value << std::endl;
            if (random_value > 0) 
                targetpos += random_value*(ma-targetpos);
            else
                targetpos += random_value * ( targetpos-mi);
            //targetpos = random_value * leng + mi;
            //targetpos += leng * random_value;
            targetpos = std::max(mi, std::min(ma, targetpos));//保证数据位于合法范围内
            time_counter += time_step;
        }

        // 3. 设置关节控制参数
        b3RobotSimulatorJointMotorArgs motorArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
        motorArgs.m_targetPosition = targetpos;  // 滑动条的目标位置
        motorArgs.m_maxTorqueValue = 5;       // force=7
        motorArgs.m_targetVelocity = 0;
        
        // 4. 应用关节控制
        G.sim_->setJointMotorControl(
            cat,
            jointIds[i],
            motorArgs
        );
    }
    G.RESET_HEIGHT = G.sim_->readUserDebugParameter(G.params["RESET_HEIGHT"]);
    G.Max_Velocity = G.sim_->readUserDebugParameter(G.params["Max_Velocity"]);

    if (G.sim_->readUserDebugParameter(G.params["Quit"]))
        return false;
    double Auto_p = G.sim_->readUserDebugParameter(G.params["Auto"]);
    if (Auto_p < 1)
        G.is_auto = false;
    else
        G.is_auto = true;
    double b_p = G.sim_->readUserDebugParameter(G.params["Allow_Reset"]);
    if (b_p<1)
        G.Allow_reset = false;
    else
        G.Allow_reset = true;
    double a_p = G.sim_->readUserDebugParameter(G.params["draw"]);
    if (a_p < 1)
        G.is_graph = false;
    else
        G.is_graph = true;
    return true;
}

struct drawer {
    
    std::vector<double> time_points, heights, lin_vels, ang_vels,
        drags, est_drags, areas,
        inertias[4], com_x, com_y,com_z, defaule;
    int cnt;
    int display_points = 3000;
    int size = 1000000;
    drawer() {
        time_points.reserve(size);
        heights.reserve(size);
        lin_vels.reserve(size);
        ang_vels.reserve(size);
        drags.reserve(size);
        est_drags.reserve(size);
        areas.reserve(size);
        for (auto& vec : inertias) vec.reserve(size);
        com_x.reserve(size);
        com_y.reserve(size);
        com_z.reserve(size);
        defaule.reserve(size);
        cnt = 0;
   }
    
    void init() {
        time_points.clear();
         heights.clear();
        lin_vels.clear();
        ang_vels.clear();
        drags.clear();
        est_drags.clear();
        areas.clear();
        for (auto& vec : inertias) vec.clear();
        com_x.clear();
        com_y.clear();
        com_z.clear();
       defaule.clear();
    }
    //时间，高度，线/角速度，|(时间),阻力，估计空气阻力，xy平面投影面积，|(时间)xyz轴和旋转方向的转动惯量,xy质心
    void add(double t, double h, double v, double w, double f, double ff, double Sxy, double Ix, double Iy, double Iz, double Iw, double x, double y,double z) {
        time_points.push_back(t);
        heights.push_back(h);
        lin_vels.push_back(v);
        ang_vels.push_back(w*10);
        drags.push_back(f);
        est_drags.push_back(ff);
        areas.push_back(Sxy);
        inertias[0].push_back(Ix);
        inertias[1].push_back(Iy);
        inertias[2].push_back(Iz);
        inertias[3].push_back(Iw);
        com_x.push_back(x);
        com_y.push_back(y);
        com_z.push_back(z);
        defaule.push_back(0.0);
    }
    
    void draw_graph() {

        int len = time_points.size();
        std::cout <<"数据点长度:" <<len << std::endl;
        double x_l = time_points[0], x_r = time_points[len - 1] ;

        plt::figure_size(1600, 1200);  // 宽度1600像素，高度1200像素
        plt::subplot2grid( 3, 2 ,  0, 0 ); // 手动定义子图网格
       // std::cout << "lj" << std::endl;
        plt::named_plot("Height", time_points, heights,"r--");
        plt::xlim(x_l,x_r);
        plt::ylabel("Height(m)");
        plt::title("T-H");
        plt::legend();
       // std::cout << "11" << std::endl;

        plt::subplot2grid(3, 2, 0, 1);
        plt::named_plot("v", time_points, lin_vels, "b--");
        plt::named_plot("w", time_points, ang_vels, "r--");
        plt::xlim(x_l, x_r);
        plt::ylabel("v(m/s)/w(10^-1 rad/s");
        plt::title("T-V/W");
        plt::legend();
       // std::cout << "22" << std::endl;

        plt::subplot2grid(3, 2, 1, 0);
       // std::cout << "kj" << std::endl;
        std::map<std::string, std::string> keywords;

        keywords["hatch"] = "-";
        plt::fill_between(time_points, drags, est_drags, keywords);
        std::cout << "   " << std::endl;
        plt::named_plot("f", time_points, drags, "b-");
        plt::named_plot("evluate-f", time_points, est_drags, "r--");
        //std::cout << "   " << std::endl;
        plt::ylabel("f/N");
        //std::cout << "   " << std::endl;
        plt::xlim(x_l, x_r);
        plt::title("T-f");
       // std::cout << "---" << std::endl;
        //std::cout << "   " << std::endl;
        plt::legend();
       // std::cout << "33" << std::endl;

        plt::subplot2grid(3, 2, 1, 1);
        plt::named_plot("Sxy", time_points, areas, "y--");
        plt::xlim(x_l, x_r);
        plt::ylabel("Sxy(m**2)");
        plt::title("T-S");
        plt::legend();

        plt::subplot2grid(3, 2, 2, 0);
        plt::named_plot("Ix", time_points, inertias[0], "r-");
        plt::named_plot("Iy", time_points, inertias[1], "g-");
        plt::named_plot("Iz", time_points, inertias[2], "y--");
        plt::named_plot("Iw", time_points, inertias[3], "b--");
        plt::xlim(x_l, x_r);
        plt::ylabel("inertia(kg/m**2)");
        plt::xlabel("Time(s)");
        plt::title("T-I");
        plt::legend();

        plt::subplot2grid(3, 2, 2, 1);
        plt::named_plot("com_x", time_points, com_x, "y--");
        plt::named_plot("com_y", time_points, com_y, "b--");
        plt::xlim(x_l, x_r);
        plt::ylabel("Pos(m)");
        plt::xlabel("Time(s)");
        plt::title("T-xy");
        plt::legend();

        plt::tight_layout();
        plt::pause(0.05);
        //std::cout << "kjs" << std::endl;
        std::string filename = "../../graph/Test/DroppingTest_"+std::to_string(cnt)+".png";
        std::cout << "文件已保存至："  << filename << std::endl;
        plt::save(filename);
        plt::show();
    }
};
int countt = 1;
void loging(Physics_SIM& G, double delta_time, double m_x, double m_y, double m_v, double m_w) {
    // --- 1. 准备评分数据 ---
    G.drag2 = std::abs(G.drag2);
    const std::vector<std::string> scores = { "猫表现良好", "猫乳臭未干", "猫表现一般", "测试无效！" };
    const std::vector<std::string> res = { "猫粉身碎骨", "猫凶多吉少", "猫并无大碍，可以挑战更高难度！","测试无效!"};

    // 计算评分 (sco)
    double m_tmp = m_v * m_w * G.drag2;
    double sco = G.RESET_HEIGHT * G.drag2 * 1e9 / std::sqrt(
        G.total_torque[0] * G.total_torque[0] +
        G.total_torque[1] * G.total_torque[1] +
        G.total_torque[2] * G.total_torque[2]
    ) / m_tmp;

    // 结果判定
    std::string m_scores = scores[0];
    std::string m_res;
    if (G.total_force_magnitude < 200) {
        m_res = res[3];
    }
    else if(G.total_force_magnitude < 1300&& G.total_force_magnitude>200) {
        m_res = res[2];
    }
    else if (G.total_force_magnitude < 2000) {
        m_res = res[1];
    }
    else {
        m_res = res[0];
    }

    // --- 2. 创建报告目录 ---

    std::string report_dir = "../../report/drop_report_num_" + std::to_string(countt) + "/";
    if (countt == 1) {
    while (fs::exists(report_dir)) {
        countt++;
        report_dir = "../../report/drop_report_num_" + std::to_string(countt) + "/";
    }
    fs::create_directory(report_dir);
    }

    // --- 3. 生成报告内容 ---
    std::string report_content =
        "落体猫运动---第" + std::to_string(G.cnt) + "次下落：\n"
        "总评：" + std::to_string(sco) + "\n"
        "单次下落高度：" + std::to_string(G.RESET_HEIGHT) + "m\n"
        "单次下落时长：" + std::to_string(delta_time) + "s\n"
        "下落累计暂停时长:" + std::to_string(G.accumulated_paused_time) + "s\n"
        "猫初始朝向：面朝y轴正方向\n"
        "猫的质量：" + std::to_string(G.Mass) + "kg\n"
        "猫的体型：长约" + std::to_string(G.cat_long) + "m，宽约" + std::to_string(G.cat_width) + "m，高约" + std::to_string(G.cat_height) + "m\n"
        "下落过程xy轴最大偏移值(区分正负)(" + std::to_string(m_x) + ", " + std::to_string(m_y) + ")\n"
        "下落最大线速度：" + std::to_string(m_v) + "m/s\n"
        "下落最大角速度：" + std::to_string(m_w) + "rad/s\n"
        "下落竖直方向最大近似线性阻力：" + std::to_string(G.drag2) + "N\n"
        "下落过程最大投影面积：" + std::to_string(G.max_Sxy) + "平方米\n"
        "落地瞬时冲击力：" + std::to_string(G.total_force_magnitude) + "N\n"
        "落地瞬时最大力矩：(" + std::to_string(G.total_torque[0]) + ", "
        + std::to_string(G.total_torque[1]) + ", "
        + std::to_string(G.total_torque[2]) + ")\n\n"
        + m_res + "\n\n"
        "注意：实际生活中猫在接触地面后还会自行调整以增加落地接触时间减小接触力,"
        "并且此处无法真实模拟猫的自然动作。此处只是展示落地瞬间猫受到的冲击力。"
        "如果冲击力为0，说明测试无效！";

    // --- 4. 写入文件 ---
    std::string file_path = report_dir + std::to_string(G.cnt) + ".txt";
    std::ofstream out_file(file_path);
    if (out_file.is_open()) {
        out_file << report_content;
        out_file.close();
        std::cout << "报告已保存至: " << file_path << std::endl;
    }
    else {
        std::cerr << "无法打开文件: " << file_path << std::endl;
    }
}

//主线程
void start_simulate(Physics_SIM& G,drawer& D)
{
    int Cat = G.model_ids_["cat"];
    // 保存初始状态
    std::cout << "开始仿真" << std::endl;
    G.sim_->setNumSimulationSubSteps(20);//设置引擎每帧求解的子步数
    G.sim_->getBasePositionAndOrientation(G.model_ids_["cat"], G.tmp.pos, G.tmp.orient);
    G.sim_->getBaseVelocity(G.model_ids_["cat"], G.tmp.v, G.tmp.w);

    G.init_orient = G.tmp.orient;
    btVector3 init_v = btVector3(0, 0, 0);
    btVector3 init_w = btVector3(0, 0, 0);
    btVector3 init_pos = G.tmp.pos;

    std::cout << "初始朝向：";
    for (int i = 0;i < 4;i++)std::cout << G.init_orient[i] << " ";
    std::cout << std::endl;

    compute_com(G, "cat");
    compute_inertia_com(G, "cat");
    std::cout << "mass:" << G.Mass << std::endl;
    std::cout << "初始质心:" << std::endl;
    for (int i = 0;i < 3;i++)std::cout << G.com << " ";
    std::cout << std::endl;
    std::cout << "初始惯性矩阵" << std::endl;
    for (int i = 0;i < 3;i++) {
        for (int j = 0;j < 3;j++)
            std::cout << G.inertia_com[i][j] << " ";
        std::cout << std::endl;
    }
    G.start_time = G.clock.getTimeInSeconds();
    G.start_time2 = G.clock.getTimeInSeconds();
    G.pause_start_time = G.start_time;
    long long frame = 0;//统计帧数

    double lastTime = G.clock.getTimeInSeconds();
    double timeAccumulator = 0.0;

    double last_v_z = 0.0;
    double last_vt = 0.0;

    btVector3 last_angular = btVector3(0, 0, 0);
    double m_px = 0.0, m_py = 0.0, m_w = 0.0, m_v = 0.0;

    int ghy = 0;
    while (G.sim_->canSubmitCommand())
    {
        double currentTime = G.clock.getTimeInSeconds();
        double deltaTime = currentTime - lastTime;

        lastTime = currentTime;
        timeAccumulator += deltaTime;

        if (timeAccumulator >= G.fixedTimeStep) {
            G.sim_->stepSimulation();  // 物理世界步进1/240s
            timeAccumulator -= G.fixedTimeStep;
        }
        else {
            double sleepTime = G.fixedTimeStep - timeAccumulator;
            if (sleepTime > 0)
                b3Clock::usleep(sleepTime * 1e6);
        }

        btVector3 base_position;
        btQuaternion base_orientation;
        btVector3 base_velocity, base_angular;

        G.sim_->getBasePositionAndOrientation(Cat, base_position, base_orientation);
        G.sim_->getBaseVelocity(Cat, base_velocity, base_angular);

        double angular_speed = base_angular.length();
        double linear_speed = base_velocity.length();
        if (base_position[0] * base_position[0] + base_position[1] * base_position[1] > m_px * m_px + m_py * m_py) { m_px = base_position[0];m_py = base_position[1]; }
        if (angular_speed > m_w)m_w = angular_speed;
        if (linear_speed > m_v)m_v = linear_speed;
        //限制物体转速
        //if (angular_speed > 2)
        //    G.sim_->resetBaseVelocity(Cat, base_velocity, btVector3(0,0,0));
        //else if(frame%5==0)
        //    last_angular = base_angular;
        //状态重置
        if (base_position[2] < G.GROUND_MIN) {
            if (G.Allow_reset|| ghy > G.stop_t) {
            // 重置位置和速度，重力
            G.sim_->setGravity(btVector3(0, 0, 0));
            G.sim_->resetBaseVelocity(Cat, init_v, init_w);
            G.sim_->resetBasePositionAndOrientation(Cat, btVector3(0, 0,G.RESET_HEIGHT), G.init_orient);
            G.is_paused = true;
            //清空状态数组
            G.sim_->getBasePositionAndOrientation(Cat, G.tmp.pos, G.tmp.orient);
            G.sim_->getBaseVelocity(Cat, G.tmp.v, G.tmp.w);
            ghy = 0;
            G.max_Sxy = 0.2;
            m_px = 0.0;
            m_py = 0.0;
            m_w = 0.0;
            m_v = 0.0;
            G.total_torque = btVector3(0.0, 0.0, 0.0);
            G.total_force_magnitude = 0.0;
            //重置暂停时间计数
            G.pause_start_time = G.clock.getTimeInSeconds();
            G.start_time2 = G.pause_start_time;
            G.accumulated_paused_time = 0.0;
            D.cnt = G.cnt;G.cnt++;
            D.init();
            }
            if (base_position[2] < 0.3&&ghy==10) {
                //物体落地
            //重置冲击力
            double cc = G.clock.getTimeInSeconds();
            std::cout << "第"<<G.cnt<<"次下落最大冲击力和力矩:" << G.total_force_magnitude << "(" << G.total_torque[0] <<" "<< G.total_torque[1] <<" "<< G.total_torque[2] << std::endl;
            std::cout << "累计暂停时长：" << G.accumulated_paused_time << "累计用时" << cc - G.accumulated_paused_time-G.start_time << std::endl;
            loging(G, cc - G.accumulated_paused_time - G.start_time2,m_px,m_py, abs(m_v),abs(m_w));
             if (G.is_graph) { D.draw_graph(); }//绘制图表
            }
            if (base_position[2] < 0.3)ghy++;
        }

      // if(!G.is_paused)std::cout << base_position[2] <<"|"<< base_velocity.length() <<"|"<<G.drag.length()<<"|"<<G.drag2 << std::endl;
       G.sim_->getBaseVelocity(Cat, base_velocity, base_angular);
       if (base_velocity.length() > G.Max_Velocity) { std::cout << "超速" << std::endl;break; }
        if(G.is_paused)G.sim_->resetBaseVelocity(Cat, init_v, init_w);
        //计算质心
        compute_com(G, "cat");
        if (frame == 0) {
            std::cout << "成功计算质心";
            for (int i = 0;i < 3;i++)std::cout << G.com[i] << std::endl;
        }
        // 计算惯性张量
        compute_inertia_com(G, "cat");
        if (frame == 0){
            std::cout << "成功计算惯性张量" << std::endl;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
                std::cout << G.inertia_com[i][j] << " ";
            std::cout << std::endl;
        }
        }
        // 计算旋转方向的转动惯量
        double rotational_inertia = 0.0;
        if (angular_speed > 1e-8) {
            btVector3 rotation_dir = base_angular.normalized();
            rotational_inertia = rotation_dir.dot(G.inertia_com * rotation_dir);
        }
        //计算投影面积
        compute_Sxy(G, Cat);
        if (frame == 0)std::cout << "成功计算投影面积" << G.Sxy<<std::endl;
        // 计算空气阻力
        compute_aerodynamic_drag(G, Cat);
        if (frame == 0) {
            std::cout << "成功计算空气阻力" << std::endl;
            for (int x = 0;x < 3;x++)std::cout << G.drag[x] << " ";
            std::cout << std::endl;
        }
            G.sim_->getBaseVelocity(Cat, base_velocity, base_angular);
            double delta_v_z = base_velocity[2] - last_v_z;
            last_v_z = base_velocity[2];
            double tmp_t = G.clock.getTimeInSeconds();
            G.drag2 = -delta_v_z / (tmp_t - last_vt) * G.Mass * 9.8 + G.Mass * 9.8;
            last_vt = tmp_t;
            //std::cout << "真实竖直方向空气阻力：" << G.drag2 << std::endl;
        
        // 评估冲击力和力矩
        if (!G.is_paused)
        evaluate_impact_forces(G, Cat);
        
        // 推送数据到deque
        //时间，高度，线/角速度，|(时间),阻力，估计空气阻力，xy平面投影面积，|(时间)xyz轴和旋转方向的转动惯量,xy质心
        //if(!G.is_paused&&frame%3==0)
        //{
        //    //std::cout << "存储数据---" << std::endl;
        //    G.sim_->getBasePositionAndOrientation(Cat, base_position, base_orientation);
        //    G.sim_->getBaseVelocity(Cat, base_velocity, base_angular);
        //    double now = G.clock.getTimeInSeconds() - G.start_time - G.accumulated_paused_time;
        //    std::unique_lock<std::mutex> lock(shared_data.mtx);//加锁
        //    shared_data.x_right = std::max(shared_data.x_right, now);
        //    std::cout << base_position[2]<< "加入数据：" << base_velocity.length()<<" "<< base_angular.length() << std::endl;
        //    shared_data.data_deque.emplace_back(  // 使用emplace_back
        //        now,base_position[2], base_velocity.length(), base_angular.length()*10,
        //        abs(G.drag2),G.drag.length(),G.Sxy,
        //        G.inertia_com[0][0], G.inertia_com[1][1], G.inertia_com[2][2], rotational_inertia,G.com[0],G.com[1]
        //    );
        //    if (shared_data.data_deque.size() > shared_data.max_data_points)
        //            shared_data.data_deque.pop_front();
        //    lock.unlock();
        //    shared_data.cv.notify_one();  // 通知绘图线程
        //}
        
         //时间，高度，线/角速度，|(时间),阻力，估计空气阻力，xy平面投影面积，|(时间)xyz轴和旋转方向的转动惯量,xy质心
        if(!G.is_paused)
        {
            //std::cout << "存储数据---" << std::endl;
            G.sim_->getBasePositionAndOrientation(Cat, base_position, base_orientation);
            G.sim_->getBaseVelocity(Cat, base_velocity, base_angular);
            double now = G.clock.getTimeInSeconds() - G.start_time2 - G.accumulated_paused_time;

            //std::cout << base_position[2]<< "加入数据：" << base_velocity.length()<<" "<< base_angular.length() << std::endl;
            D.add( 
                now,base_position[2], base_velocity.length(), base_angular.length(),
                abs(G.drag2),G.drag.length(),G.Sxy,
                G.inertia_com[0][0], G.inertia_com[1][1], G.inertia_com[2][2], rotational_inertia,G.com[0],G.com[1], G.com[2]
            );
        }

        if(frame==0)
        std::cout << "开始处理键盘事件" << std::endl;
        //处理键盘事件
        handleKeyboardEvents(G, Cat);
        // 更新相机视角
        G.sim_->getBasePositionAndOrientation(Cat, base_position, base_orientation);
        G.sim_->resetDebugVisualizerCamera(
            G.cam->distance,
            G.cam->yaw,
            G.cam->pitch,
            base_position + btVector3(0, 0, G.cat_height + G.cam->offset)
        );
        if (frame == 0)
            std::cout << "开始应用滑动条参数" << std::endl;
        bool is_quit= controlJointsWithSliders(G, Cat,frame);

        if (!is_quit)break;

        frame++;
    }
}

//绘图
void graph3(SharedData& shared_data,drawer& D) {
    plt::clf();  // 清除当前图形
    plt::figure_size(1200, 800);
    plt::ion();//开启交互模式

    plt::detail::_interpreter::get(); // 确保解释器初始化
    PyObject* pyplot = PyImport_ImportModule("matplotlib.pyplot");

    while (true) {
        std::cout << "绘图线程已经准备就绪:" << std::endl;
        std::unique_lock<std::mutex> lock(shared_data.mtx);
        shared_data.cv.wait(lock, [&] {
            return !shared_data.data_deque.empty() || !shared_data.sim_running;
            });

        if (!shared_data.sim_running && shared_data.data_deque.empty()) {
            break;
        }
        std::cout << "开始绘图" << std::endl;
        auto& data_deque = shared_data.data_deque;
        const size_t data_size = data_deque.size();
       double x_r = shared_data.x_right;
       double delta_time = shared_data.delta_time;
        double x_l = std::max(0.0, x_r - shared_data.delta_time);
        
        // 提取数据（只取最近的display_points个点）
        size_t start_idx = (data_size > D.display_points) ? data_size - D.display_points : 0;

        for (size_t i = start_idx, idx = 0; i < data_size; ++i, ++idx) {
            const auto& point = data_deque[i];
            D.time_points[idx] = (std::get<0>(point));
            D.heights[idx] = (std::get<1>(point));
            D.lin_vels[idx] = (std::get<2>(point));
            D.ang_vels[idx] = (std::get<3>(point));
            D.drags[idx] = (std::get<4>(point));
            D.est_drags[idx] = (std::get<5>(point));
            D.areas[idx] = (std::get<6>(point));
            D.inertias[0][idx] = (std::get<7>(point));
            D.inertias[1][idx] = (std::get<8>(point));
            D.inertias[2][idx] = (std::get<9>(point));
            D.inertias[3][idx] = (std::get<10>(point));
            D.com_x[idx] = (std::get<11>(point));
            D.com_y[idx] = (std::get<12>(point));
        }
        lock.unlock();//解锁
        std::cout << "data_size" << data_size <<" "<<x_r<<" "<< delta_time << std::endl;
        std::cout << "数据复制完毕" << std::endl;

        // 更新图表
        plt::clf();

        std::cout << "jjjk" << std::endl;
        // ========== 子图1: 高度和速度 ==========
        plt::subplot(1, 1, 1);//1行1列，选中第一个
        std::cout << "jjjk" << std::endl;
        // 高度（左轴）
        plt::named_plot("Height", D.time_points, D.heights, "b-");
        plt::ylabel("Height (m)",{{"color","#1f77b4"}});
        plt::ylim(0,1000); 
        std::cout << "jjj" << std::endl;

        PyObject* ax = PyObject_CallMethod(pyplot, "gca", nullptr);
        PyObject* twin_ax = PyObject_CallMethod(ax, "twinx", nullptr);
        if (!twin_ax) throw std::runtime_error("Failed to create twinx");
        PyObject_CallMethod(twin_ax, "set_ylim", "(ff)", 0,60);
        PyObject_CallMethod(ax, "set_xlim", "(ff)",x_l,x_r);

        // 在次 y 轴上绘制速度
        PyObject_CallMethod(twin_ax, "plot", "(OOO)",
            to_python(D.time_points), to_python(D.lin_vels), Py_BuildValue("s", "g-"));
        PyObject_CallMethod(twin_ax, "plot", "(OOO)",
            to_python(D.time_points), to_python(D.ang_vels), Py_BuildValue("s", "r--"));
        PyObject_CallMethod(twin_ax, "set_ylabel", "(s)", "Velocity (m/s / rad/s)");
        std::cout << "图例" << std::endl;
        // 图例和标题
        plt::legend();
        plt::title("Motion Profile (Height & Velocity)");
        plt::grid(true);

        //// ========== 子图2: 阻力和面积 ==========
        //plt::subplot(3, 1, 2);

        //// 阻力和估计阻力（左轴）
        //plt::named_plot("Drag Force", time_points, drags, "b-");
        //plt::named_plot("Estimated Drag", time_points, est_drags, "r--");
        //plt::ylabel("Drag Force (N)", { {"color", "#1f77b4"} });
        //plt::ylim(0, 600);
        //// 使用原生Python命令创建次y轴
        //plt::detail::_interpreter::get();
        ////PyObject* pyplot = PyImport_ImportModule("matplotlib.pyplot");
        //PyObject* ax2 = PyObject_CallMethod(pyplot, "gca", nullptr);
        //PyObject* twin_ax2 = PyObject_CallMethod(ax2, "twinx", nullptr);
        //if (!twin_ax2) throw std::runtime_error("Failed to create twinx for subplot 2");
        //PyObject_CallMethod(twin_ax2, "set_ylim", "(ff)", 0, 0.3);
        //PyObject_CallMethod(ax2, "set_xlim", "(ff)", x_l, x_r);

        //// 在次y轴上绘制面积
        //PyObject_CallMethod(twin_ax2, "plot", "(OOO)",
        //    to_python(time_points),
        //    to_python(areas),
        //    Py_BuildValue("s", "g-."));
        //PyObject_CallMethod(twin_ax2, "set_ylabel", "(s)", "Area (m²)");
        //
        //// 设置颜色
        //PyObject_CallMethod(twin_ax2, "yaxis", "O", Py_BuildValue("{s:s}", "labelcolor", "#8c564b"));

        //// 图例和标题
        //plt::legend();
        //plt::title("Aerodynamic Properties");
        //plt::grid(true);

        //// ========== 子图3: 惯性质心 ==========
        //plt::subplot(3, 1, 3);

        //// 转动惯量（左轴）
        //const std::string inertia_labels[4] = { "Ixx", "Iyy", "Izz", "Irot" };
        //const std::string inertia_colors[4] = { "#e377c2", "#7f7f7f", "#bcbd22", "#17becf" };

        //for (int i = 0; i < 4; ++i) {
        //    plt::named_plot(inertia_labels[i], time_points, inertias[i],
        //        "linestyle='-', color='" + inertia_colors[i] + "', linewidth=1.5");
        //}
        //plt::ylabel("Moment of Inertia (kg·m²)");
        //plt::ylim(-1000, 1000);
        //// 使用原生Python命令创建次y轴
        //PyObject* ax3 = PyObject_CallMethod(pyplot, "gca", nullptr);
        //PyObject* twin_ax3 = PyObject_CallMethod(ax3, "twinx", nullptr);
        //if (!twin_ax3) throw std::runtime_error("Failed to create twinx for subplot 3");
        //PyObject_CallMethod(twin_ax3, "set_ylim", "(ff)",-3, 3);
        //PyObject_CallMethod(ax3, "set_xlim", "(ff)", x_l, x_r);

        //// 在次y轴上绘制质心坐标
        //PyObject_CallMethod(twin_ax3, "plot", "(OOO)",
        //    to_python(time_points),
        //    to_python(com_x),
        //    Py_BuildValue("{s:s,s:s,s:i}", "color", "#1a55FF", "linestyle", "--", "linewidth", 1.5));

        //PyObject_CallMethod(twin_ax3, "plot", "(OOO)",
        //    to_python(time_points),
        //    to_python(com_y),
        //    Py_BuildValue("{s:s,s:s,s:i}", "color", "#FF1a55", "linestyle", ":", "linewidth", 1.5));

        //PyObject_CallMethod(twin_ax3, "set_ylabel", "(s)", "COM Position (m)");

        //// 图例和标题
        //plt::legend();
        //plt::title("Inertial Properties & Center of Mass");
        //plt::grid(true);
        std::cout << "进行" << std::endl;
        // 减少引用计数
        Py_DECREF(twin_ax);
        /*Py_DECREF(twin_ax2);
        Py_DECREF(twin_ax3);*/
        Py_DECREF(ax);
       /* Py_DECREF(ax2);
        Py_DECREF(ax3);*/
        Py_DECREF(pyplot);

        // 调整布局
        plt::tight_layout();
        plt::pause(0.05);
    }

    PyRun_SimpleString("plt.ioff()");//嵌入python代码关闭交互模式
    plt::show();
}

void graph2(SharedData& shared_data, drawer& D) {
    plt::detail::_interpreter::get();  // 初始化Python解释器
    plt::figure_size(1200, 800);
    plt::ion();

    // 初始化主Y轴
    plt::Plot plot1("Height", "b-");
    plot1.update(D.time_points, D.heights);

    // 初始化次Y轴
    PyObject* pyplot = PyImport_ImportModule("matplotlib.pyplot");
    if (!pyplot) throw std::runtime_error("Failed to import matplotlib.pyplot");

    PyObject* ax = PyObject_CallMethod(pyplot, "gca", nullptr);
    PyObject* twin_ax = PyObject_CallMethod(ax, "twinx", nullptr);
    PyObject_CallMethod(ax, "set_xlim", "(ff)", 0.0, shared_data.delta_time);
    PyObject_CallMethod(ax, "set_xlabel", "(m)", "Height(m)");
    PyObject_CallMethod(twin_ax, "set_ylabel", "(s)", "Velocity (m/s /1e-1 rad/s)");
    PyObject_CallMethod(ax, "set_ylim", "(ff)", 0.0, 1000.0);
    PyObject_CallMethod(twin_ax, "set_ylim", "(ff)", 0.0, 60.0);

    // 创建次Y轴曲线
    PyObject* lin_vel_line = nullptr;
    PyObject* ang_vel_line = nullptr;

    while (true) {
        // 等待数据

        std::unique_lock<std::mutex> lock(shared_data.mtx);
        shared_data.cv.wait(lock, [&] {
            return !shared_data.data_deque.empty() || !shared_data.sim_running;
            });

        if (!shared_data.sim_running && shared_data.data_deque.empty()) break;

        // 提取最新数据
        auto& data_deque = shared_data.data_deque;
        const size_t data_size = data_deque.size();
        double x_r = shared_data.x_right;
        double delta_time = shared_data.delta_time;
        double x_l = std::max(0.0, x_r - shared_data.delta_time);
        size_t start_idx = (data_size > D.display_points) ? data_size - D.display_points : 0;

        for (size_t i = start_idx, idx = 0; i < data_size; ++i, ++idx) {
            const auto& point = data_deque[i];
            D.time_points[idx] = (std::get<0>(point));
            D.heights[idx] = (std::get<1>(point));
            D.lin_vels[idx] = (std::get<2>(point));
            D.ang_vels[idx] = (std::get<3>(point));
            D.drags[idx] = (std::get<4>(point));
            D.est_drags[idx] = (std::get<5>(point));
            D.areas[idx] = (std::get<6>(point));
            D.inertias[0][idx] = (std::get<7>(point));
            D.inertias[1][idx] = (std::get<8>(point));
            D.inertias[2][idx] = (std::get<9>(point));
            D.inertias[3][idx] = (std::get<10>(point));
            D.com_x[idx] = (std::get<11>(point));
            D.com_y[idx] = (std::get<12>(point));
        }
        lock.unlock();

        // 更新主Y轴
        plot1.update(D.time_points, D.heights);

        PyObject_CallMethod(ax, "set_ylabel", "(s)", "Height (m)");
        PyObject_CallMethod(ax, "set_ylim", "(ff)", 0.0, 1000.0);
        PyObject_CallMethod(ax, "set_xlim", "(ff)", x_l, x_r);
        // 更新次Y轴曲线
        PyObject* py_time_points = to_python(D.time_points);
        PyObject* py_lin_vels = to_python(D.lin_vels);
        PyObject* py_ang_vels = to_python(D.ang_vels);

        if (!lin_vel_line) {
            lin_vel_line = PyObject_CallMethod(twin_ax, "plot", "(OO)O", py_time_points, py_lin_vels, Py_BuildValue("s", "g-"));
            ang_vel_line = PyObject_CallMethod(twin_ax, "plot", "(OO)O", py_time_points, py_ang_vels, Py_BuildValue("s", "r--"));
        }
        else {
            // 更新现有曲线
            PyObject_CallMethod(PyObject_GetItem(lin_vel_line, PyLong_FromLong(0)), "set_data", "(OO)", py_time_points, py_lin_vels);
            PyObject_CallMethod(PyObject_GetItem(ang_vel_line, PyLong_FromLong(0)), "set_data", "(OO)", py_time_points, py_ang_vels);
        }

        PyObject_CallMethod(twin_ax, "set_xlim", "(ff)", x_l, x_r);
        PyObject_CallMethod(twin_ax, "set_ylim", "(ff)", 0.0, 60.0);
        PyObject_CallMethod(twin_ax, "set_ylabel", "(s)", "Velocity (m/s /1e-1 rad/s)");
        // 装饰元素
        plt::legend();
        plt::title("Motion Profile (Height & Velocity)");
        plt::grid(true);
        plt::pause(0.05);

        // 清理临时Python对象
        Py_XDECREF(py_time_points);
        Py_XDECREF(py_lin_vels);
        Py_XDECREF(py_ang_vels);
    }

    // 释放资源
    Py_XDECREF(lin_vel_line);
    Py_XDECREF(ang_vel_line);
    Py_DECREF(twin_ax);
    Py_DECREF(ax);
    Py_DECREF(pyplot);
    plt::show();
}

int main()
{   
    drawer D;
    Physics_SIM G;
    G.window_init();
    start_simulate(G,D);

    //double a;
    //std::cout << "请输入时间坐标轴长度：" << std::endl;
    //std::cin >> a;

    //SharedData shared_data;  // 共享数据结构
    //drawer D(shared_data.max_data_points);
    //shared_data.delta_time = a;
    //shared_data.x_right = a;

    // 启动绘图线程
    /*std::thread plot_thread(graph, std::ref(shared_data),std::ref(D));*/

    // 
    //double x = 5.0;
    //for (int i = 0;i < 10000;i++,x++) {
    //    D.add(x, x, x, x, x, x, x, x, x, x, x, x, x,x);
    //}
    //D.draw_graph();
    //D.init();
    //for (int i = 0;i < 10000;i++, x++) {
    //    D.add(x, x, x, x, x, x, x, x, x, x, x, x, x, x);
    //}
    //D.draw_graph();
    //std::vector<double> x;
    //std::vector<double> y;
    //for (double i = 1.0; i < 10;i++) {
    //    x.push_back(i);
    //    y.push_back(2 * i);
    //}
    //plt::plot(x, y);
    //plt::show();
    //x.clear();
    //y.clear();
    //for (double i = 1.0; i < 10;i++) {
    //    x.push_back(i);
    //    y.push_back(2 * i);
    //}
    //plt::plot(x, y);
    //plt::show();
   // plt::show();
    //shared_data.sim_running = false;  // 通知绘图线程结束
    //shared_data.cv.notify_one();      // 确保绘图线程能退出

    //// 等待绘图线程安全退出
    //if (plot_thread.joinable()) {
    //    plot_thread.join();
    //}

   //int Cat = G.model_ids_["cat"];
   ////G.sim_->resetBasePositionAndOrientation(Cat, btVector3(0, 0, 100),btQuaternion(0,0,0,0) );
   //
   // G.sim_->setGravity(btVector3(0, 0, 0));
   // long long frame = 1;//统计帧数
   // G.sim_->setNumSimulationSubSteps(20);
   // double lastTime = G.clock.getTimeInSeconds();
   // double timeAccumulator = 0.0;
   // int ff = 0;
   // while (G.sim_->canSubmitCommand()){
   //     ff++;
   //     if(ff==500)G.sim_->setGravity(btVector3(0, 0, -9.8));
   //     btVector3 base_position;
   //     btQuaternion base_orientation;
   //     btVector3 base_velocity, base_angular;
   //     G.sim_->getBasePositionAndOrientation(Cat, base_position, base_orientation);
   //     G.sim_->getBaseVelocity(Cat, base_velocity, base_angular);
   //     if (base_velocity[2] < 0.2 && base_position[2] < 5) {
   //         G.sim_->resetBaseVelocity(Cat, btVector3(0, 0, 0), btVector3(0, 0, 0));
   //         G.sim_->resetBasePositionAndOrientation(Cat, btVector3(0, 0, 100), G.init_ori);
   //     }
   //     std::cout << "v:" << base_velocity[2] << std::endl;
   //     double currentTime = G.clock.getTimeInSeconds();
   //     double deltaTime = currentTime - lastTime;

   //     lastTime = currentTime;
   //     timeAccumulator += deltaTime;
   //     if (deltaTime > G.fixedTimeStep) deltaTime = G.fixedTimeStep;
   //     if (timeAccumulator >= G.fixedTimeStep) {
   //         G.sim_->stepSimulation();  // 物理世界步进1/240s
   //         timeAccumulator -= G.fixedTimeStep;
   //     }
   //     else {
   //         double sleepTime = G.fixedTimeStep - timeAccumulator;
   //         if (sleepTime > 0)
   //             b3Clock::usleep(sleepTime * 1e6);
   //     }
   //     // 更新相机视角
   //     btVector3 bpos;
   //     btQuaternion bori;
   //     G.sim_->getBasePositionAndOrientation(Cat, bpos, bori);
   //     std::cout << bpos[2] << std::endl;
   //     G.sim_->resetDebugVisualizerCamera(
   //         G.cam->distance,
   //         G.cam->yaw,
   //         G.cam->pitch,
   //         bpos + btVector3(0, 0, G.cat_height + G.cam->offset)
   //     );
   //     //处理关节调试
   //     bool is_quit = controlJointsWithSliders(G, Cat);
   //    
   //     if (!is_quit)break;
   // }
    //if (G.is_paused)G.accumulated_paused_time += G.now - G.pause_start_time;
    //std::cout << "accumulated_paused_time:" << G.accumulated_paused_time << std::endl;
    //std::cout << "now:" << G.now<< std::endl;
   
    //system("pause");
}
