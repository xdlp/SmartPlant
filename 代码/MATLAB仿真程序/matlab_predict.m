clc;
clear;
a1 = textread ('����ʪ�����ݵ�1.txt');%��������ʪ�����ݵ�
b1 = textread('�����¶����ݵ�.txt');%���ػ����¶����ݵ�
c1 = textread('����ʪ�����ݵ�.txt');%���ػ���ʪ�����ݵ�
a1 = a1(1:110,1);%������ʪ��ֵ����
b1 = b1(1:110,1);
c1 = c1(1:110,1);
a1 = dtrend(a1);


figure(1)
plot(a1,'r-');

figure(2)
subplot(2,3,1)
autocorr(a1);
title('subplot1:����ʪ������غ���');
subplot(2,3,4)
parcorr(a1);
title('subplot2:����ʪ��ƫ����غ���');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,2)
autocorr(b1(1:110));
title('subplot2:�����¶�����غ���');
subplot(2,3,5)
parcorr(b1(1:110));
title('subplot5:�����¶�ƫ����غ���');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,3)
autocorr(c1(1:110));
title('subplot3:����ʪ������غ���');
subplot(2,3,6)
parcorr(c1(1:110));
title('subplot6:����ʪ��ƫ����غ���');


%�ж�����ʪ�������Ƿ�Ϊʱ��ƽ������
Y=a1(1:110);
[H1,PValue1,TestStat1,CriticalValue]=adftest(Y,'alpha',0.3);%��HֵΪ1��������ƽ�ȣ�������������εĲ������
X=b1;
[H2,PValue2,TestStat2,CriticalValue]=adftest(X,'alpha',0.5);%��HֵΪ1��������ƽ�ȣ�������������εĲ������
Z=c1;
[H3,PValue3,TestStat3,CriticalValue]=adftest(Z,'alpha',0.5);%��HֵΪ1��������ƽ�ȣ�������������εĲ������

for i=1:999
  if H1==1
    break;
  else
DX=diff(Y,i); %���в��
     [H1,PValue,TestStat,CriticalValue]=adftest(Y,'alpha',0.05);
  end
end
figure(3)
subplot(2,1,1)
autocorr(Y);
subplot(2,1,2)
parcorr(Y);

%ģ�Ͷ���
%mvnrnd (0,1,100 );%������ֵΪ0������Ϊ1�����ݳ���Ϊ100�İ��������ݵ�

b_c = [b1,c1];
z=iddata(Y,b_c);

test=[];
for i=1:10    %����ʪ�Ƚ״ι���
    for j=1:10    %������״ι���
        for k=1:10   %�����¶Ƚ״ι���
            for h=1:10  %����ʪ�Ƚ״ι���
            
                M = armax(z,'na',i,'nb',[k,h],'nc',j,'nk',[0,0]);
                AIC = aic(M);
                test = [test;i,k,h,j,AIC];  
            end
        end
    end
end
for v=1:size(test,1)
    if test(v,5)==min(test(:,5))   %ѡ��AICֵ��С��ģ��,ȷ�����������Ľ״�
        na = test(v,1);
        nb = [test(v,2),test(v,3)];
        
        nc = test(v,4);
        break;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��Ϲ���
m1=armax(z,'na',na,'nb',nb,'nc',nc,'nk',[0,0]);       %armax(p,q,[p_test q_test])��ӦAICֵ��С
figure(4)
e=resid(m1,z);      %������в����
plot(e);
%����в������غ�ƫ��غ���
figure(5)
subplot(2,1,1)
autocorr(e.OutputData)     %һ�ײ������z����غ���ͼMA(q),����ˮƽ0.95
subplot(2,1,2)
autocorr(e.OutputData)     %һ�ײ������zƫ��غ���ͼAR(p),����ˮƽ0.95
%Ԥ�����
pp=predict(m1,z,1);
figure(6)
%plot(z,'r',pp,'b');
plot(pp,'r');
hold on;
plot(a1,'g');
hold off;

po=pp.OutputData;
%��������
for i=1:10
    A_data(i,:)=po(i,:);
end
xlswrite('soil_wetness.xls',A_data);




