clc;
clear;
a1 = textread ('土壤湿度数据点1.txt');%加载土壤湿度数据点
b1 = textread('环境温度数据点.txt');%加载环境温度数据点
c1 = textread('环境湿度数据点.txt');%加载环境湿度数据点
a1 = a1(1:110,1);%将土壤湿度值纵排
b1 = b1(1:110,1);
c1 = c1(1:110,1);
a1 = dtrend(a1);


figure(1)
plot(a1,'r-');

figure(2)
subplot(2,3,1)
autocorr(a1);
title('subplot1:土壤湿度自相关函数');
subplot(2,3,4)
parcorr(a1);
title('subplot2:土壤湿度偏自相关函数');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,2)
autocorr(b1(1:110));
title('subplot2:环境温度自相关函数');
subplot(2,3,5)
parcorr(b1(1:110));
title('subplot5:环境温度偏自相关函数');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,3)
autocorr(c1(1:110));
title('subplot3:环境湿度自相关函数');
subplot(2,3,6)
parcorr(c1(1:110));
title('subplot6:环境湿度偏自相关函数');


%判断土壤湿度序列是否为时间平稳序列
Y=a1(1:110);
[H1,PValue1,TestStat1,CriticalValue]=adftest(Y,'alpha',0.3);%若H值为1，则序列平稳，否则进行最高五次的差分运算
X=b1;
[H2,PValue2,TestStat2,CriticalValue]=adftest(X,'alpha',0.5);%若H值为1，则序列平稳，否则进行最高五次的差分运算
Z=c1;
[H3,PValue3,TestStat3,CriticalValue]=adftest(Z,'alpha',0.5);%若H值为1，则序列平稳，否则进行最高五次的差分运算

for i=1:999
  if H1==1
    break;
  else
DX=diff(Y,i); %进行差分
     [H1,PValue,TestStat,CriticalValue]=adftest(Y,'alpha',0.05);
  end
end
figure(3)
subplot(2,1,1)
autocorr(Y);
subplot(2,1,2)
parcorr(Y);

%模型定阶
%mvnrnd (0,1,100 );%产生均值为0，方差为1，数据长度为100的白噪声数据点

b_c = [b1,c1];
z=iddata(Y,b_c);

test=[];
for i=1:10    %土壤湿度阶次估计
    for j=1:10    %噪声项阶次估计
        for k=1:10   %环境温度阶次估计
            for h=1:10  %环境湿度阶次估计
            
                M = armax(z,'na',i,'nb',[k,h],'nc',j,'nk',[0,0]);
                AIC = aic(M);
                test = [test;i,k,h,j,AIC];  
            end
        end
    end
end
for v=1:size(test,1)
    if test(v,5)==min(test(:,5))   %选择AIC值最小的模型,确定各个变量的阶次
        na = test(v,1);
        nb = [test(v,2),test(v,3)];
        
        nc = test(v,4);
        break;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%拟合过程
m1=armax(z,'na',na,'nb',nb,'nc',nc,'nk',[0,0]);       %armax(p,q,[p_test q_test])对应AIC值最小
figure(4)
e=resid(m1,z);      %拟合做残差分析
plot(e);
%检验残差的自相关和偏相关函数
figure(5)
subplot(2,1,1)
autocorr(e.OutputData)     %一阶差分序列z自相关函数图MA(q),置信水平0.95
subplot(2,1,2)
autocorr(e.OutputData)     %一阶差分序列z偏相关函数图AR(p),置信水平0.95
%预测过程
pp=predict(m1,z,1);
figure(6)
%plot(z,'r',pp,'b');
plot(pp,'r');
hold on;
plot(a1,'g');
hold off;

po=pp.OutputData;
%保存数据
for i=1:10
    A_data(i,:)=po(i,:);
end
xlswrite('soil_wetness.xls',A_data);




