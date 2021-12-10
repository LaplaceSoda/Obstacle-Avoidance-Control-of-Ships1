%% ��Ŀ����
%     ��֪:��ʼ�㡢�յ�λ���Լ��켣f(x1),f(x2)
%     �ѵ�:ͨ���켣����+�ٶ����ʱ�򷽳�
%     Ҫʹ�ò���ײ��������ֻ��Ҫ������ʱ�̱�֤����������ڰ�ȫ���뼴��
%     �������ϣ����ٷ�Χһ��Ϊ0-20��,��ȫ����һ��Ϊ2����
%     Ҫ�������ٶȣ�ֻ��Ҫʱ����̼���
clc;
clear all;
%% ������֪����
%     Xi,Yi    ����ʱ��A��B��������
%     Fi       A��B���Ĺ켣����
%     Xoi,Yoi  A,B����ʼλ��
%     Xti,Yti  A,B������λ��
syms Xa Ya Xb Yb;
M1 = str2num(input("����A����ʼ���꣺",'s'));
Xoa = M1(1);
Yoa = M1(2);
M2 = str2num(input("����A����ֹ���꣺",'s'));
Xta = M2(1);
Yta = M2(2);
M3 = str2num(input("����B����ʼ���꣺",'s'));
Xob = M3(1);
Yob = M3(2);
M4 = str2num(input("����B����ֹ���꣺",'s'));
Xtb = M4(1);
Ytb = M4(2);
assume(Xoa<=Xa<=Xta);
assume(Xob<=Xb<=Xtb);
for i = 1:5
    Sa = input("����A���Ĺ켣���̣�",'s');
    Sb = input("����B���Ĺ켣���̣�",'s');
    Fa = inline(Sa);
    Fb = inline(Sb);
    if Fa(Xoa) == Yoa & Fa(Xta)==Yta & Fb(Xob) == Yob & Fb(Xtb) == Ytb
        disp("��֪�����������");
        break;
    else
        disp("�켣���������������������");
    end 
end
%% ѭ����
syms x;
Va = 1:20;
Vb = 1:20;
t = 2000*ones(20,20);
for i=1:20
    for j=1:20
        % ·�������ֿɵõ��˶���Xi����ʱ��
        Ta(x) = (int(sqrt(1+(diff(Fa(x)))^2),Xoa,x))/Va(i);
        Tb(x) = (int(sqrt(1+(diff(Fb(x)))^2),Xob,x))/Vb(j);
        % ��Ta��Tb�󷴺����ɵõ�X(t),�����Ƚϵͣ��������еĿ��ܲ�����
        % �˴�����solve���а�ȫ�����У��
        tmin =min((int(Fa(x),Xoa,Xta))/Va(i),(int(Fb(x),Xob,Xtb))/Vb(j))
        for k=0:1:tmin
            Xa = solve(k == Ta(x),x);
            Ya = Fa(Xa);
            Xb = solve(k == Tb(x),x);
            Yb = Fb(Xb);
            % ִ�а�ȫ������ж�
            if (Xb-Xa)^2+(Yb-Xa)^2<=2^2
                continue;
            end
            % ���ܰ�ȫ������������Ӧ�ٶ��µ�ʱ�䣬���д���
            t(i,j)=max((int(Fa(x),Xoa,Xta))/Va(i),(int(Fb(x),Xob,Xtb))/Vb(j));
        end
    end
end
%% ������С��ʱ�䣬��ȡ���ŷ����������
Tmin = min(min(t))
[Vat,Vbt]=find(Tmin==t)
