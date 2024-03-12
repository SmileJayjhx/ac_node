主程序功能：
    - 执行serer_cmd；
    - 反馈serer_encoder；
    - 零点校正；
    - 手自动切换
main.cpp 全向车
main_duigao_rebot.cpp rebot车型，注意车体需要同时接收到运动控制和叉齿控制才是有效，只运动也需要给叉发0，在叉齿控制以实现。
main_duigao.cpp 如意车型，转向和行走都是柯蒂斯
main_floor.cpp 如意车型，转向和行走是两套协议
main_mima.cpp 如意车型，转向和行走是两套协议


