%% 题目分析
%     已知:起始点、终点位置以及轨迹f(x1),f(x2)
%     难点:通过轨迹方程+速度求解时域方程
%     要使得不相撞，本质上只需要在任意时刻保证两船距离大于安全距离即可
%     查阅资料，航速范围一般为0-20节,安全距离一般为2海里
%     要求得最佳速度，只需要时间最短即可
clc;
clear all;
%% 输入已知条件
%     Xi,Yi    任意时刻A，B船的坐标
%     Fi       A，B船的轨迹函数
%     Xoi,Yoi  A,B船初始位置
%     Xti,Yti  A,B船结束位置
syms Xa Ya Xb Yb;
M1 = str2num(input("输入A船起始坐标：",'s'));
Xoa = M1(1);
Yoa = M1(2);
M2 = str2num(input("输入A船终止坐标：",'s'));
Xta = M2(1);
Yta = M2(2);
M3 = str2num(input("输入B船起始坐标：",'s'));
Xob = M3(1);
Yob = M3(2);
M4 = str2num(input("输入B船终止坐标：",'s'));
Xtb = M4(1);
Ytb = M4(2);
assume(Xoa<=Xa<=Xta);
assume(Xob<=Xb<=Xtb);
for i = 1:5
    Sa = input("输入A船的轨迹方程：",'s');
    Sb = input("输入B船的轨迹方程：",'s');
    Fa = inline(Sa);
    Fb = inline(Sb);
    if Fa(Xoa) == Yoa & Fa(Xta)==Yta & Fb(Xob) == Yob & Fb(Xtb) == Ytb
        disp("已知条件输入完毕");
        break;
    else
        disp("轨迹函数输入错误，请重新输入");
    end 
end
%% 循环体
syms x;
Va = 1:20;
Vb = 1:20;
t = 2000*ones(20,20);
for i=1:20
    for j=1:20
        % 路径弧积分可得到运动到Xi处的时间
        Ta(x) = (int(sqrt(1+(diff(Fa(x)))^2),Xoa,x))/Va(i);
        Tb(x) = (int(sqrt(1+(diff(Fb(x)))^2),Xob,x))/Vb(j);
        % 对Ta、Tb求反函数可得到X(t),但精度较低，反函数有的可能不存在
        % 此处采用solve进行安全距离的校验
        tmin =min((int(Fa(x),Xoa,Xta))/Va(i),(int(Fb(x),Xob,Xtb))/Vb(j))
        for k=0:1:tmin
            Xa = solve(k == Ta(x),x);
            Ya = Fa(Xa);
            Xb = solve(k == Tb(x),x);
            Yb = Fb(Xb);
            % 执行安全距离的判断
            if (Xb-Xa)^2+(Yb-Xa)^2<=2^2
                continue;
            end
            % 对能安全的条件，求解对应速度下的时间，进行储存
            t(i,j)=max((int(Fa(x),Xoa,Xta))/Va(i),(int(Fb(x),Xob,Xtb))/Vb(j));
        end
    end
end
%% 查找最小的时间，获取最优方案，并输出
Tmin = min(min(t))
[Vat,Vbt]=find(Tmin==t)
