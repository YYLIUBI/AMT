clear all;close all;clc;
CAL_AMP=79.588;%2500mVp*680/(10000+680)=159mVpp
[FileName,PathName] = uigetfile('*.TS*','Select the Calibration-file','MultiSelect','on');%选择需要处理的数据文件
data={};
data_in={};
figure(1);
for a=1:1:length(FileName)    
     fid = fopen(FileName{a}, 'rb');
    switch FileName{a}(strfind(FileName{a},'.TS')+3)%SHML
    case 'S'
        fs(a)=24000;
        fin(a)=240;
        fn(a)=25;
        LEN(a)=24000;
        LEN_pass(a)=533;
        T(a)=50;
        fdelta(a)=LEN(a)*fin(a)/fs(a);
        ts(a)=2;
        fseek(fid,16,'bof');%skip 16B
    case 'H'
        fs(a)=2400;
        fin(a)=24;
        fn(a)=22;
%         LEN(a)=24000;
        LEN(a)=12000;
        LEN_pass(a)=533;
        T(a)=50;
        fdelta(a)=LEN(a)*fin(a)/fs(a); 
        ts(a)=3;
        fseek(fid,16,'bof');%skip 16B
    case 'M'
        fs(a)=150;
        fin(a)=2.5;
        fn(a)=14;
        LEN(a)=1200;
        LEN_pass(a)=181;
        T(a)=30;
        fdelta(a)=LEN(a)*fin(a)/fs(a);
        ts(a)=4;
        fseek(fid,16+52*32,'bof');%skip 16B
    case 'L'
        fs(a)=15;
        fin(a)=15/160;
        fn(a)=36;
        LEN(a)=1600;
        LEN_pass(a)=151;
        T(a)=80;
        fdelta(a)=LEN(a)*fin(a)/fs(a);   
        ts(a)=5;
        fseek(fid,16,'bof');%skip 16B
    end 
    data = [data,fread(fid,[32,1024*1024/32],'uchar')]; 
    for j=1:1:8
        data_in=[data_in,dealdata((data{a}(3+4*(j-1),:)*256*256*256+data{a}(2+4*(j-1),:)*256*256+data{a}(1+4*(j-1),:)*256))];
    end  
    for j=1:2:LEN(a)/T(a)+1
        for i=((j-1)*T(a)+1):1:(T(a)*j)
            square(i)=-CAL_AMP;
        end
        for i=(T(a)*j+1):1:(T(a)*(j+1))
            square(i)=CAL_AMP;
        end
    end
    for i=1:1:8 
        data_fft{i+8*(a-1)}=data_in{i+8*(a-1)}(LEN_pass(a)+1:LEN(a)+LEN_pass(a))';
        y_in{i+8*(a-1)}=fft(data_fft{i+8*(a-1)},LEN(a));
    end
    data_out{a}=square(1:LEN(a))';
    y_out{a}=fft(data_out{a},LEN(a));
    subplot(length(FileName),1,a);
    plot(data_fft{1+(a-1)*8},'r*-');hold on;
    plot(data_fft{2+(a-1)*8},'g*-');hold on;
    plot(data_fft{3+(a-1)*8},'b*-');hold on;
    plot(data_fft{5+(a-1)*8},'k*-');hold on;
    plot(data_fft{6+(a-1)*8},'y*-');hold on;
    plot(data_fft{7+(a-1)*8},'m*-');hold on;  
    plot(data_out{a},'c*-');grid on;   
 %   legend('Ex','Ey','Ez','Hx','Hy','Hz');
end
% scrsz = get(0,'ScreenSize');set(gcf,'Position',scrsz);
figure(2)
for a=1:1:length(FileName)
    f{a}=0:fs(a)/LEN(a):fs(a)/2-fs(a)/LEN(a);
    f{a}=f{a}'; 
    for i=1:1:8
        amp_in{i+8*(a-1)}=abs(y_in{i+8*(a-1)}(1:LEN(a)/2));
        phase_in{i+8*(a-1)}=angle(y_in{i+8*(a-1)}(1:LEN(a)/2))*180/pi;
    end
    amp_out{a}=abs(y_out{a}(1:LEN(a)/2));
    phase_out{a}=angle(y_out{a}(1:LEN(a)/2))*180/pi;    
    subplot(length(FileName),2,2*a-1);      
    stem(f{a},amp_in{1+8*(a-1)},'r*-');hold on;
    stem(f{a},amp_in{2+8*(a-1)},'g*-');hold on;
    stem(f{a},amp_in{3+8*(a-1)},'b*-');hold on;
    stem(f{a},amp_in{5+8*(a-1)},'k*-');hold on;
    stem(f{a},amp_in{6+8*(a-1)},'y*-');hold on;
    stem(f{a},amp_in{7+8*(a-1)},'m*-');hold on;
    stem(f{a},amp_out{a},'c*-');grid on;
 %   legend('Ex','Ey','Ez','Hx','Hy','Hz');
    subplot(length(FileName),2,2*a); 
    stem(f{a},phase_in{1+8*(a-1)},'r*-');hold on;
    stem(f{a},phase_in{2+8*(a-1)},'g*-');hold on;
    stem(f{a},phase_in{3+8*(a-1)},'b*-');hold on;
    stem(f{a},phase_in{5+8*(a-1)},'k*-');hold on;
    stem(f{a},phase_in{6+8*(a-1)},'y*-');hold on;
    stem(f{a},phase_in{7+8*(a-1)},'m*-');hold on;
    stem(f{a},phase_out{a},'c*-');grid on;    
   % legend('Ex','Ey','Ez','Hx','Hy','Hz');
end
% scrsz = get(0,'ScreenSize');set(gcf,'Position',scrsz);
figure(3);
for a=1:1:length(FileName)
    for i=1:1:8
        amp_outA{i+8*(a-1)}=zeros(1,fn(a));realA{i+8*(a-1)}=zeros(1,fn(a));
        phase_outA{i+8*(a-1)}=zeros(1,fn(a));imageA{i+8*(a-1)}=zeros(1,fn(a));
        for j=1:1:fn(a)
            amp_outA{i+8*(a-1)}(j)=amp_in{i+8*(a-1)}((2*j-1)*fdelta(a)+1);     
            phase_outA{i+8*(a-1)}(j)=phase_in{i+8*(a-1)}((2*j-1)*fdelta(a)+1);
            realA{i+8*(a-1)}(j)=(y_in{i+8*(a-1)}((2*j-1)*fdelta(a)+1)+conj(y_in{i+8*(a-1)}((2*j-1)*fdelta(a)+1)))/2;
            imageA{i+8*(a-1)}(j)=-((y_in{i+8*(a-1)}((2*j-1)*fdelta(a)+1)-conj(y_in{i+8*(a-1)}((2*j-1)*fdelta(a)+1)))/2).*1i; 
        end
    end
    for j=1:1:fn(a)
        f_out{a}(j)=(2*j-1)*fin(a);
        amp_outB{a}(j)=amp_out{a}((2*j-1)*fdelta(a)+1);
        phase_outB{a}(j)=phase_out{a}((2*j-1)*fdelta(a)+1);    
        realB{a}(j)=(y_out{a}((2*j-1)*fdelta(a)+1)+conj(y_out{a}((2*j-1)*fdelta(a)+1)))/2;    
        imageB{a}(j)=-((y_out{a}((2*j-1)*fdelta(a)+1)-conj(y_out{a}((2*j-1)*fdelta(a)+1)))/2).*1i;   
    end
    for i=1:1:8
        d_amp{i+8*(a-1)}=amp_outA{i+8*(a-1)}./amp_outB{a};
        d_phase{i+8*(a-1)}=phase_outA{i+8*(a-1)}-phase_outB{a};
        for j=1:1:fn(a)
            if abs(d_phase{i+8*(a-1)}(j)) > 180
                d_phase{i+8*(a-1)}(j) = abs(d_phase{i+8*(a-1)}(j)) - 360;
            end
        end 
    end
   subplot(211);
    loglog(f_out{a},d_amp{1+8*(a-1)},'r^-');hold on;
    loglog(f_out{a},d_amp{3+8*(a-1)},'b^-');hold on;
    loglog(f_out{a},d_amp{2+8*(a-1)},'g^-');hold on;
    loglog(f_out{a},d_amp{5+8*(a-1)},'k*-');hold on;
    loglog(f_out{a},d_amp{6+8*(a-1)},'y*-');hold on;
    loglog(f_out{a},d_amp{7+8*(a-1)},'m*-');grid on;  
    xlabel('f,Hz');
    ylabel('Gain,V/V');
    legend('Ex','Ey','Ez','Hx','Hy','Hz');
    set(gca,'FontSize',16,'FontName','Arial');
    set(get(gca,'XLabel'),'FontSize',16,'FontName','Arial');
    set(get(gca,'YLabel'),'FontSize',16,'FontName','Arial');
    subplot(212);
    semilogx(f_out{a},d_phase{1+8*(a-1)},'r^-');hold on;  
    semilogx(f_out{a},d_phase{2+8*(a-1)},'g^-');hold on;
    semilogx(f_out{a},d_phase{3+8*(a-1)},'b^-');hold on;
    semilogx(f_out{a},d_phase{5+8*(a-1)},'k*-');hold on;
    semilogx(f_out{a},d_phase{6+8*(a-1)},'y*-');hold on;
    semilogx(f_out{a},d_phase{7+8*(a-1)},'m*-');grid on;
    xlabel('f,Hz');
    ylabel('Phase,Degree');
    legend('Ex','Ey','Ez','Hx','Hy','Hz');
    set(gca,'FontSize',16,'FontName','Arial');
    set(get(gca,'XLabel'),'FontSize',16,'FontName','Arial');
    set(get(gca,'YLabel'),'FontSize',16,'FontName','Arial');
end
% scrsz = get(0,'ScreenSize'); set(gcf,'Position',scrsz);

% fidri=fopen('SHML.asc','w');
fidri=fopen([FileName{a}(1:4),'H.asc'],'w');
for k=1:1:length(FileName)
    a=5-k;
    for j=1:1:fn(a)
        fprintf(fidri,'%d, %010.5f, %02d,',ts(a),f_out{a}(j),2*j-1);
        for i=1:1:3
            d_real{i+8*(a-1)}(j)=(realA{i+8*(a-1)}(j)*realB{a}(j)+imageA{i+8*(a-1)}(j)*imageB{a}(j))/(realB{a}(j)^2+imageB{a}(j)^2);
            d_image{i+8*(a-1)}(j)=(imageA{i+8*(a-1)}(j)*realB{a}(j)-realA{i+8*(a-1)}(j)*imageB{a}(j))/(realB{a}(j)^2+imageB{a}(j)^2);
            fprintf(fidri,'% 4.5f,% 4.5f,% 4.5f,% 4.5f,% 4.5f,% 4.5f,',...
            realA{i+8*(a-1)}(j),imageA{i+8*(a-1)}(j),realB{a}(j),imageB{a}(j),d_real{i+8*(a-1)}(j),d_image{i+8*(a-1)}(j));        
        end
        for i=5:1:7
            d_real{i+8*(a-1)}(j)=(realA{i+8*(a-1)}(j)*realB{a}(j)+imageA{i+8*(a-1)}(j)*imageB{a}(j))/(realB{a}(j)^2+imageB{a}(j)^2);
            d_image{i+8*(a-1)}(j)=(imageA{i+8*(a-1)}(j)*realB{a}(j)-realA{i+8*(a-1)}(j)*imageB{a}(j))/(realB{a}(j)^2+imageB{a}(j)^2);
            fprintf(fidri,'% 4.5f,% 4.5f,% 4.5f,% 4.5f,% 4.5f,% 4.5f,',...
            realA{i+8*(a-1)}(j),imageA{i+8*(a-1)}(j),realB{a}(j),imageB{a}(j),d_real{i+8*(a-1)}(j),d_image{i+8*(a-1)}(j));        
        end
        fprintf(fidri,'\n');
    end 
end
fclose(fidri);