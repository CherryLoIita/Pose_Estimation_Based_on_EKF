
function main()


    clc; 
    
    
    %.............................................................................
    % API 文件
    MyApi = APImtrn4010_v05();                      % init API
    % 数据文件
    file = '.\Datasets\aDataUsr_007b.mat';
    % 读取数据文件
    r = MyApi.b.LoadDataFile(file);                  % load dataset of interest
    %.............................................................................

    
    nL = r.n(2);                   % number of LiDAR event

    XXXgt = zeros(3,nL);
    XXXe = zeros(3,nL);

    discrepancy_XY = zeros(1,nL);
    error_X = zeros(1,nL);
    error_Y = zeros(1,nL);
    disrepancy_head = zeros(1,nL);
    
    std_X = zeros(1,nL);
    std_Y = zeros(1,nL);
    std_heading = zeros(1,nL);

    TTT = zeros(2,nL);
    

    if r.ok<1, return ; end;       % r.ok<1 : there are issues with the data file.
    % possible issues:  file does not exist, or data structure is not correct. 
    % you may exploit the returned info, which is about the length of the dataset
    %nE=r.n(1) ;  % number of events in this dataset/playback session.
    %nL=r.n(2) ;  % number of LiDAR events ....   : this may be useful to predefine certain Buffers we need in Project1.
    
    % MyApi.b.Rst();                                   % API function to reset playback pointer  
    
    
    
    
    %.......................................................................................................................
    %  API 函数

    ReadNextEvent=MyApi.RdE;                           % :read next event
    ConvertRangestoXY=MyApi.b.ConvertRangestoXY;       % :convert scan ranges (polar) to carterian points in LiDAR's CF (*)
    GetCurrentGT = MyApi.gcgt;                         % : get current actual pose.
    AnimateShape = MyApi.AniShape;                     % : helper function for basic animation
    GetRangeAndIntensityFromRawScan = MyApi.b.GetRangeAndIntensityFromRawScan;      % parses raw LidAR scans. (*)
    % (*) : you must implement your own version, as part of project 1.
    GegGoodClusters = MyApi.b.FindSmallSegmentsFS;
    ExePlotSegments = MyApi.b.ShSe;
    % Extra information (map of walls and poles in the area,etc) 
    UsefulInfo=MyApi.b.GetInfo();
    %......................................................................................................................

    

    %-----------------------------------------------------------------------------------------------------------------------
    % figure(20)(GCF)
    
    % GCF中 Car 的行车路线

    % GCF中 GT Car 和 预测Car 的坐标位置的 plot handle
    GT = MyApi.b.GetGroundTruth();
    hhGCF = MkFigurePlotPoseAndGT(20,GT);
    
    % GCF中 GT Car 和 预测Car 的朝向(三角箭头轮廓)的 plot handle
    hCarGT = plot(0,0,'r-');
    hCarE = plot(0,0,'b-');

    
    % 墙壁 和 landmark
    Context=UsefulInfo.Context;
    SomeBrownColor = [205, 127, 50]/255;  % RGB, sort of brown...
    plot(Context.Walls(1,:),Context.Walls(2,:),'-','linewidth',2,'color',SomeBrownColor);       %plot walls 
    plot(Context.Landmarks(1,:),Context.Landmarks(2,:),'m*');                                   %plot navigation poles.
    Landmarks = Context.Landmarks;

    % GCF中 2个LiDAR 的 所有扫描点 的 plot handle (GT & 预测)
    hhPinGCFgt = [plot(0,0,'b.'), plot(0,0,'k.')];
    hhPinGCFes = [plot(0,0,'b.'), plot(0,0,'g.')];
    
    % GCF中 2个LiDAR 的 所有OOI 的 plot handle
    hOOI1 = plot(0,0,'b+', MarkerSize=10);
    hOOI2 = plot(0,0,'k+', MarkerSize=10);


    % GCF中 2个LiDAR 的 所有OOI 与 landmark的连线 的 plot handle
    hs_ooi = [plot(0,0,'-m','linewidth',1), plot(0,0,'-m','linewidth',1)];

    % GCF中 2个LiDAR 的 所有OOI 与 Car 的连线 的 plot handle
    hs_ctes = [plot(0,0,'--k', linewidth=0.5), plot(0,0,'--k', linewidth=0.5)];

    

    hold off;                  % no more plots to be added in this figure. 
    %-----------------------------------------------------------------------------------------------------------------------
    
    
    
    %-------------------------------------------------------------------------------------------------------------------------------------------------
    % figure(21)(Polar坐标系 & Cartesian坐标系)

    [hh1,hooi] = MkFigureToPlotLiDAR1sScan(21);
    

    %-------------------------------------------------------------------------------------------------------------------------------------------------
    
   

    %-------------------------------------------------------------------------------------------------------------------------------------------------
    % API 创建 Menu
    MyApi.b.MkMenuControl(20);
    MyApi.b.MkMenuControl(21);
    %-------------------------------------------------------------------------------------------------------------------------------------------------
    
    
    
    % 前后2个LiDAR 的 车载位置
    Lidar1CFG =  UsefulInfo.LidarsCfg.Lidar1 ;    % installation info about Lidar#1
    Lidar2CFG =  UsefulInfo.LidarsCfg.Lidar2 ;    % installation info about Lidar#2

    Lidar1PoseInCar = [Lidar1CFG.Lx; Lidar1CFG.Ly; Lidar1CFG.Alpha];
    Lidar2PoseInCar = [Lidar2CFG.Lx; Lidar2CFG.Ly; Lidar2CFG.Alpha];
    
    
    % Car 的 初始姿态
    pose0 = UsefulInfo.pose0;                    % platform's initial pose. 
    
    
    % 固定变量，默认变量
    UsefulInfo=[];  Context=[];     % variables not needed anymore, in this example.
    % global v_edit;  v_edit = 0;
    % global w_edit;  w_edit = 0.1;
    global pause_time;  pause_time = 0.00;
    Std_Speed = 0.1;
    Std_Gyro = 3/180*pi;
    R = 0.15^2;

    
    % 姿态预测初始化
    t0=0;
    Xe = pose0;     P = zeros(3);
    vw=[0;0];       Pu = zeros(2);   Pu(1,1) = Std_Speed^2;    Pu(2,2) = Std_Gyro^2;


    %-------------------------------------------------------------------------------------------------------------------------------------------------
    % 循环读取 event
    while 1 
        
        e=ReadNextEvent();              % read next available event (event type, time and data)
        
        t = double(e.t)*0.0001;         % time of current event, in seconds.
        dt=t-t0;
        t0=t;                           % time elapsed since last event.  It will be useful in the implementations of certain processing, in Project 1.


        % 预测 Car 姿态
        if(dt>0) 
            [Xe,P] = Prediction(Xe, P, vw, Pu, dt);

        elseif(dt<0)
            Xe = pose0;     P = zeros(3);
            vw=[0;0];       Pu = zeros(2);   Pu(1,1) = Std_Speed^2;    Pu(2,2) = Std_Gyro^2;
        end
        


        % Dispatch actions, based on the type of sensor event.
        switch(e.ty)
            
            case 0   % -----------------------------  "end of trip" event ---------------------------
                disp('Found the END') ; break ;   % data type = 0 --> END, no more events. end of trip. so END LOOP. 
            
            case 1  % -----------------------------  event due to LiDARs measurement
                    
                    % LiDAR event, ==> data will have a raw scan from LIDAr1, and anther scan from LiDAR2,
                    
                    % 获取 2个LiDAR 的 原始扫描数据(uint16): 包含 intensities 和 range 信息
                    d = e.d;      % data is 301x2    [scan1,scan2], class uint16. d(:,1) is scan1, and d(:,2) is scan2

                    % ---------------------------------------------------------------------------------
                    % 当前 event 的序号
                    eLc = e.i(2);   
                    % 计时开始
                    tic0 = tic();
                    
                    % 获取 Car GT姿态(xy, heading rate)
                    poseGTNow = GetCurrentGT();
                    
                    
                    % 获取 当前LiDAR event 的 range(301×1) 和 intensity(301×1)
                    [intensisties1,rangesM1] = MyGetRangeAndIntensityFromRawScan(d(:,1));
                    [intensisties2,rangesM2] = MyGetRangeAndIntensityFromRawScan(d(:,2));
        
                    % [rangesCm,intensisties1,rangesM] = GetRangeAndIntensityFromRawScan(RawScan);
                    % rangesCm: ranges in cm, class uint16
                    % rangesM: ranges in metres, class single (a.k.a."float" in C/C++)
                    % intensities: class uint8
        
                    
                    % 获取 高反射点(intensity>0) (单位:1-301)
                    ii1 = find(intensisties1>0);
        
                    
                    % 获取 符合条件的 OOI
                    % iiab: 每个OOI的左右边缘(单位:1-301)
                    % props: 每个OOI的 1:预估宽度 2:距离 3:角度
                    % numOOIs: OOI的个数
                    dL = 0.8;
                    [iiab1, props1, numOOIs1] = GegGoodClusters(rangesM1, dL, 0.5);
                    [iiab2, props2, numOOIs2] = GegGoodClusters(rangesM2, dL, 0.5);
                    
                    
                    % 获取 所有OOI 的 Polar坐标信息
                    r01 = props1(:,2);     a01 = props1(:,3);
                    r02 = props2(:,2);     a02 = props2(:,3);
                    
                    % 前后雷达的 所有OOI坐标: Polar -> Cartesian(LiDAR)
                    k = pi/180;
                    [xOOI1, yOOI1] = PolarToCarte(r01, a01*k);
                    [xOOI2, yOOI2] = PolarToCarte(r02, a02*k);


                    
                    % 前后雷达的 所有OOI坐标: LiDAR -> GCF
                    [xOOI1g, yOOI1g] = LiDAR_to_GCF(xOOI1, yOOI1, Lidar1PoseInCar, Xe);
                    [xOOI2g, yOOI2g] = LiDAR_to_GCF(xOOI2, yOOI2, Lidar2PoseInCar, Xe);
                    
                    
                    
                    % 获取 Useful OOI 和 对应的landmark，用于 EKF Update
                    UOOIs1 = []; UOOIs2 = [];
                    UOOIs1cx = []; UOOIs2cx = []; UOOIs1cy = []; UOOIs2cy = [];
                    ULandmarks1x = []; ULandmarks2x = []; ULandmarks1y = []; ULandmarks2y = [];
                    UOOIs_Car = [];
                    ULandmark_GCF = [];


                    if ~isempty(props1)
                        [UOOIs1, ULandmarks1] = GetUseful([xOOI1g;yOOI1g], Landmarks);

                        UOOIs1x = UOOIs1(1,:);  UOOIs1y = UOOIs1(2,:);
                        ULandmarks1x = ULandmarks1(1,:);    ULandmarks1y = ULandmarks1(2,:);

                        [UOOIs1cx, UOOIs1cy] = GCF_to_Car(UOOIs1x, UOOIs1y, Xe);
                    end

                    if ~isempty(props2)
                        [UOOIs2, ULandmarks2] = GetUseful([xOOI2g;yOOI2g], Landmarks);

                        UOOIs2x = UOOIs2(1,:);  UOOIs2y = UOOIs2(2,:);
                        ULandmarks2x = ULandmarks2(1,:);    ULandmarks2y = ULandmarks2(2,:);

                        [UOOIs2cx, UOOIs2cy] = GCF_to_Car(UOOIs2x, UOOIs2y, Xe);
                    end


                    UOOIs_Car = [UOOIs1cx,UOOIs2cx;UOOIs1cy,UOOIs2cy];
                    ULandmark_GCF = [ULandmarks1x,ULandmarks2x;ULandmarks1y,ULandmarks2y];


                    
                    
                    
                    % EKF Update
                    if ~isempty(ULandmark_GCF)

                        r_measure = sqrt(sum(UOOIs_Car.^2, 1));

                        [Xe,P] = EKFUpDate(Xe,P,ULandmark_GCF,r_measure,R);
                        
                    end

   
                    


                    % 将 扫描点的 range(301×1) 转换为 xy坐标
                    [xx1,yy1] = MyConvertRangestoXY(rangesM1);
                    [xx2,yy2] = MyConvertRangestoXY(rangesM2);
        

                    % 将 扫描点的xy坐标: LiDAR -> Car -> GCF(GT & 预测)

                    [xx1g, yy1g] = LiDAR_to_GCF(xx1, yy1, Lidar1PoseInCar, poseGTNow);
                    [xx2g, yy2g] = LiDAR_to_GCF(xx2, yy2, Lidar2PoseInCar, poseGTNow);

                    [xx1ge, yy1ge] = LiDAR_to_GCF(xx1, yy1, Lidar1PoseInCar, Xe);
                    [xx2ge, yy2ge] = LiDAR_to_GCF(xx2, yy2, Lidar2PoseInCar, Xe);
                    % ---------------------------------------------------------------------------------
                    

                    
                    % 数据计算消耗的时间
                    eTLa = toc(tic0)*1000;

                    

                    % ---------------------------------------------------------------------------------
                    % 刷新 figure(21)(Polar & Cartesian)中 所有扫描点
                    RefreshPLotsLiDAR1sScan(hh1,rangesM1,ii1,xx1,yy1); 
                    set(hooi,'xdata',xOOI1,'ydata',yOOI1);
        
                    % 刷新 figure(20)(GCF)中 Car的坐标位置(GT & 预测)
                    set(hhGCF(1),'xdata',poseGTNow(1),'ydata',poseGTNow(2));
                    set(hhGCF(2),'xdata',Xe(1),'ydata',Xe(2));
                    
                    % 刷新 figure(20)(GCF)中 Car的朝向(三角箭头轮廓)(GT & 预测)
                    AnimateShape(hCarGT,1,poseGTNow);
                    AnimateShape(hCarE,1,Xe);
        

                    % 刷新 figure(20)(GCF)中 前后2个LiDAR 的 所有扫描点(GT & 预测)
                    set(hhPinGCFgt(1),'xdata',xx1g,'ydata',yy1g);
                    set(hhPinGCFgt(2),'xdata',xx2g,'ydata',yy2g);
                    
                    set(hhPinGCFes(1),'xdata',xx1ge,'ydata',yy1ge);
                    set(hhPinGCFes(2),'xdata',xx2ge,'ydata',yy2ge);



                    % % 刷新 figure(20)(GCF)中 前后2个LiDAR 的 Useful OOI 与 landmark 的连线
                    % % 刷新 figure(20)(GCF)中 前后2个LiDAR 的 Useful OOI 与 Car 的连线
                    if ~isempty(UOOIs1)
                    %   前LiDAR: Useful OOI
                        set(hOOI1,'xdata',UOOIs1x,'ydata',UOOIs1y);
                        % set(hOOI1,'xdata',xOOI1g,'ydata',yOOI1g);
                    % 
                    %   前LiDAR: OOI -- landmark    
                        ExePlotSegments(hs_ooi(1),[ULandmarks1x;ULandmarks1y],[UOOIs1x; UOOIs1y]);

                    %   前LiDAR: OOI -- Car
                        ExePlotSegments(hs_ctes(1),[Xe(1);Xe(2)],[UOOIs1x; UOOIs1y]);

                    else
                        set(hOOI1,'xdata',-10,'ydata',-10);
                        ExePlotSegments(hs_ooi(1),[-10;-10],[-10;-10]);
                        ExePlotSegments(hs_ctes(1),[-10;-10],[-10;-10]);
                    end
                    % 
                    if ~isempty(UOOIs2)
                    %   后LiDAR: Useful OOI
                        set(hOOI2,'xdata',UOOIs2x,'ydata',UOOIs2y);
                        % set(hOOI2,'xdata',xOOI2g,'ydata',yOOI2g);
                    %   后LiDAR: OOI -- landmark   
                        ExePlotSegments(hs_ooi(2),[ULandmarks2x;ULandmarks2y],[UOOIs2x; UOOIs2y]);

                    %   后LiDAR: OOI -- Car
                        ExePlotSegments(hs_ctes(2),[Xe(1);Xe(2)],[UOOIs2x; UOOIs2y]);

                    else
                        set(hOOI2,'xdata',-10,'ydata',-10);
                        ExePlotSegments(hs_ooi(2),[-10;-10],[-10;-10]);
                        ExePlotSegments(hs_ctes(2),[-10;-10],[-10;-10]);
                    end
                    % ---------------------------------------------------------------------------------



                    %----------------------------------------------------------------------------------
                    % 数据计算 + 图像显示 消耗的总时间
                    eTLb = toc(tic0)*1000;
                    % 将 2段处理时间 存储到 TTT[2×nL]
                    TTT(1:2,eLc) = [eTLa;eTLb];
                    % 将 Car 的 GCF坐标(GT & 预测) 分别存储到 XXXgt[3×nL] 和 XXXe[3×nL]
                    XXXgt(:,eLc) = poseGTNow;
                    XXXe(:,eLc) = Xe;
                    % 将 distance 差值 存储到 discrepancy_XY[1×nL]
                    d = Xe(1:2) - poseGTNow(1:2);
                    d = d.*d;
                    discrepancy_XY(1,eLc) = sqrt( sum(d(1,:) + d(2,:)) );
                    % 将 X坐标 差值 存储到 error_X[1×nL]
                    error_X(1,eLc) = Xe(1) - poseGTNow(1);
                    % 将 Y坐标 差值 存储到 error_Y[1×nL]
                    error_Y(1,eLc) = Xe(2) - poseGTNow(2);
                    % 将 heading rate 差值 存储到 dhr[1×nL]
                    disrepancy_head(1,eLc) = ( Xe(3) - poseGTNow(3) )*180/pi;
                    
                    % 将 X坐标 的标准差存储到 std_X[1×nL]
                    if P(1,1)>0
                        std_X(1,eLc) = sqrt(P(1,1));
                    else
                        std_X(1,eLc) = sqrt(-P(1,1));
                    end
                    % 将 Y坐标 的标准差存储到 std_Y[1×nL]
                    if P(2,2)>0
                        std_Y(1,eLc) = sqrt(P(2,2));
                    else
                        std_Y(1,eLc) = sqrt(-P(2,2));
                    end
                    % 将 Phi 的标准差存储到 std_heading[1×nL]
                    if P(3,3)>0
                        std_heading(1,eLc) = sqrt(P(3,3))/pi*180;
                    else
                        std_heading(1,eLc) = sqrt(-P(3,3))/pi*180;
                    end
                    %---------------------------------------------------------------------------------



                    pause(pause_time);
        
                    
        
            case 2  % -----------------------------  Event due to dead-reckoning measurement
                    % dead reckoning (longitudinal velocity (aka speed) and angular rate)
                    vw = e.d;  % 2x1,  [ v ; w ]
                    % vw(1) = vw(1) + v_edit*randn(1);
                    % vw(2) = vw(2) + w_edit*randn(1);


            
            otherwise %-----------------------------
                % ? % we do not consider other sensors in Project 1, so we ignore those. 
        
        
        end     % end switch
        
    

        
    end     % end loop
    %-------------------------------------------------------------------------------------------------------------------------------------------------
    
    
    
    %-------------------------------------------------------------------------------------------------------------------------------------------------
    % figure(22): distance 差值
    % figure(22); clf();
    % 
    % plot([1:nL], discrepancy_XY);
    % grid on;
    % axis([ 1, nL, min(discrepancy_XY)-0.1*(max(discrepancy_XY)-min(discrepancy_XY)), max(discrepancy_XY)+0.1*(max(discrepancy_XY)-min(discrepancy_XY))] );
    % title('Discrepancy of XY (m)');
    % xlabel('iteration');
    % ylabel('Discrepancy of XY (m)');


    % figure(23): X坐标 和 标准差
    figure(23); clf();

    plot([1:nL], error_X,LineWidth=1);  hold on;
    plot([1:nL], std_X,'c');  hold on;    plot([1:nL], -std_X,'c');  hold on;
    plot([1:nL], 2*std_X,'g');  hold on;    plot([1:nL], -2*std_X,'g');  hold on;
    plot([1:nL], 3*std_X,'k');  hold on;    plot([1:nL], -3*std_X,'k');  hold on;
    
    
    grid on;
    % axis( [ 1, nL, min(disrepancy_head) - 0.1*(max(disrepancy_head)-min(disrepancy_head)), max(disrepancy_head) + 0.1*(max(disrepancy_head)-min(disrepancy_head))] );
    title('Error X (m)');
    xlabel('iteration');
    ylabel('Error X (m)');

    
    % figure(24): Y坐标 和 标准差
    figure(24); clf();

    plot([1:nL], error_Y,LineWidth=1);  hold on;
    plot([1:nL], std_Y,'c');  hold on;    plot([1:nL], -std_Y,'c');  hold on;
    plot([1:nL], 2*std_Y,'g');  hold on;    plot([1:nL], -2*std_Y,'g');  hold on;
    plot([1:nL], 3*std_Y,'k');  hold on;    plot([1:nL], -3*std_Y,'k');  hold on;
    grid on;
    % axis( [ 1, nL, min(disrepancy_head) - 0.1*(max(disrepancy_head)-min(disrepancy_head)), max(disrepancy_head) + 0.1*(max(disrepancy_head)-min(disrepancy_head))] );
    title('Error Y (m)');
    xlabel('iteration');
    ylabel('Error Y (m)');
    

    % figure(25): 角度差值 和 标准差
    figure(25); clf();

    plot([1:nL], disrepancy_head,LineWidth=1);  hold on;
    plot([1:nL], std_heading,'c');  hold on;    plot([1:nL], -std_heading,'c');  hold on;
    plot([1:nL], 2*std_heading,'g');  hold on;    plot([1:nL], -2*std_heading,'g');  hold on;
    plot([1:nL], 3*std_heading,'k');  hold on;    plot([1:nL], -3*std_heading,'k');  hold on;
    grid on;
    % axis( [ 1, nL, min(disrepancy_head) - 0.1*(max(disrepancy_head)-min(disrepancy_head)), max(disrepancy_head) + 0.1*(max(disrepancy_head)-min(disrepancy_head))] );
    title('Error Phi (degrees)');
    xlabel('iteration');
    ylabel('Error Phi (degrees)');
    
    
    % figure(26): 程序执行时间
    % figure(26); clf();
    % 
    % plot([1:nL], TTT(1,:), 'r');
    % hold on;
    % grid on;
    % plot([1:nL], TTT(2,:), 'c');
    % % axis( [1, nL, 0, max(TTT(1,:),TTT(2,:))] );
    % title('Processing time');
    % xlabel('iteration');
    % ylabel('Processing time (ms)');
    %-------------------------------------------------------------------------------------------------------------------------------------------------



    MyApi.b.PrintStsA(0);   % print some statistics, for the teaching staff.



end     % end main()
%-------------------------------------------------------------------------------------------------------------------------------------------------



%-------------------------------------------------------------------------------------------------------------------------------------------------
% figure(20): GCF中 Car的路径 和 Car坐标位置的 plot handle
function h = MkFigurePlotPoseAndGT(ff,GTposes)
    
    figure(ff); clf();

    % 小车的路径（绿线）
    x=GTposes(1,:);
    y=GTposes(2,:);
    plot(x,y,'g');                   % static plot: the GT path.
    
    hold on;
    
    h(1) = plot(0,0,'r*');              % 显示 GCF中 Car GT坐标位置的 plot handle
    h(2) = plot(0,0,'b*');              % 显示 GCF中 Car 预测坐标位置的 plot handle
    
    grid on ; 
    axis equal;
    axis([-5,20,-5,20]);
    xlabel('X'); ylabel('Y');
    title('Global CF');

end
%-----------------------------------------------------------------



% -----------------------------------------------------------------------------------------------------
% fugure(21): Polar坐标系 和 Cartesian坐标系 的 plot handle
function [hh,hooi] = MkFigureToPlotLiDAR1sScan(figu)
    
    figure(figu); clf();

    % 21-1 polar 坐标系
    subplot(211) ; 
    
    aa = ((0:300)*0.5)-75;
    h1=plot(aa,aa*0,'.b');      % h1: 显示 polar坐标系中 所有扫描点 的 plot handle
    
    axis([-75,75,-10,30]);       
    hold on; 
    
    h2 = plot(0,0,'+r');          % h2: 显示 polar坐标系中 OOI 的 plot handle
    
    title('LiDAR_1 (in polar)');  grid on ;
    xlabel('azimuth (degrees)');
    ylabel('range (m)');

    
    % 21-2 Cartesian 坐标系
    subplot(212) ; 

    h3=plot(0,0,'.b');          % h3: 显示 Cartesian坐标系中 所有扫描点 的 plot handle
    
    hold on;
    
    hooi = plot(0,0,'ro',MarkerSize=15);

    axis([-5,15,-10,10]);
    grid on;
    title('LiDAR_1 (pointing ahead)');
    xlabel('X   :   Ahead ==>   ');
    
    hh=[h1,h2,h3];

end
% -----------------------------------------------------------------------------------------------------



%......................................................................
% 刷新 polar坐标系 和 Cartesian坐标系 的所有扫描点
function RefreshPLotsLiDAR1sScan(hh,r,ii,xx,yy)

    h1 = hh(1); h2 = hh(2);  h3 = hh(3);    
    
    % polar坐标系 的 所有扫描点
    set(h1,'ydata',r);                      % all raw points,
    
    % polar坐标系 的 OOI
    aa=single(ii-1)*0.5-75;
    set(h2,'xdata',aa,'ydata',r(ii));       % selected ones, using indexes in array ii    
    
    % Cartesian坐标系 的 所有扫描点
    set(h3,'xdata',xx,'ydata',yy);     

end
%......................................................................



%......................................................................
% 原始 uint16 数据处理，得到 intensities 和 ranges
function [intensities,ranges] = MyGetRangeAndIntensityFromRawScan(scan)

    for i = 1:size(scan,1)

        d = single(scan(i));

        if d >= 49152
            intensities(i) = 3;
            ranges(i) = (d-49152)/100;
        end

        if d<=16384
            intensities(i) = 0;
            ranges(i) = d/100;
        end

    end
    
    intensities = intensities';
    ranges = ranges';

end
%......................................................................



%......................................................................
% 将 ranges数据 转换为 Cartesian坐标
function [xx,yy] = MyConvertRangestoXY(ranges)

    a = ([0:300]*0.5 - 75)/180*pi;

    for i = 1:301
        xx(i) = ranges(i)*cos(a(i));
        yy(i) = ranges(i)*sin(a(i));
    end
    
    xx = xx';
    yy = yy';
end

%......................................................................



%......................................................................
% Polar坐标 -> Cartesian坐标
function [x, y] = PolarToCarte(range, angle)
    
    x = range .* cos(angle);
    y = range .* sin(angle);

end
%......................................................................



%......................................................................
% 扫描点的xy坐标: LiDAR -> Car
function [xxc, yyc] = LiDAR_to_Car(xx, yy, LidarPoseInCar)

    scan_in_LiDAR = [xx'; yy'];
    
    T_LiDAR_x = LidarPoseInCar(1);
    T_LiDAR_y = LidarPoseInCar(2);
    
    h_LiDAR = LidarPoseInCar(3);
    R = [cos(h_LiDAR), -sin(h_LiDAR);
         sin(h_LiDAR),  cos(h_LiDAR)];
    
    scan_in_Car = R * scan_in_LiDAR;
    xxc = scan_in_Car(1,:) + T_LiDAR_x;
    yyc = scan_in_Car(2,:) + T_LiDAR_y;

end
%......................................................................



%......................................................................
% 扫描点的xy坐标: Car -> GCF
function [xxg, yyg] = Car_to_GCF(xxc, yyc, poseCarNow)
    
    % Car -> GCF
    scan_in_Car = [xxc; yyc];
    
    T_Car_x = poseCarNow(1);
    T_Car_y = poseCarNow(2);
    
    h_Car = poseCarNow(3);
    R = [cos(h_Car), -sin(h_Car);
         sin(h_Car),  cos(h_Car)];
    
    scan_in_GCF = R * scan_in_Car;
    xxg = scan_in_GCF(1,:) + T_Car_x;
    yyg = scan_in_GCF(2,:) + T_Car_y;

end
%......................................................................



%......................................................................
% 扫描点的xy坐标: LiDAR -> Car -> GCF
function [xxg, yyg] = LiDAR_to_GCF(xx, yy, LidarPoseInCar, poseCarNow)

   % LiDAR -> Car
   [xxc, yyc] = LiDAR_to_Car(xx, yy, LidarPoseInCar);
   
   scan_in_Car = [xxc; yyc];
   
   % Car -> GCF
   [xxg, yyg] = Car_to_GCF(xxc, yyc, poseCarNow);

end
%......................................................................



%......................................................................
% 扫描点的xy坐标: GCF -> Car
function [xxc, yyc] = GCF_to_Car(xxg, yyg, poseCarNow)
    
    T_Car_x = poseCarNow(1);
    T_Car_y = poseCarNow(2);
    
    h_Car = poseCarNow(3);
    R = [cos(h_Car), -sin(h_Car);
         sin(h_Car),  cos(h_Car)];
    
    xxg = xxg - T_Car_x;
    yyg = yyg - T_Car_y;

    scan_in_Car = inv(R) * [xxg;yyg];

    xxc = scan_in_Car(1,:);
    yyc = scan_in_Car(2,:);

end
%......................................................................



%......................................................................
% 预测模型
function [Xe,P] = Prediction(Xe, P, vw, Pu, dt)

    % Expected value : Xe
    dL = vw(1)*dt;
    Xe(1:2) = Xe(1:2) + dL*[cos(Xe(3)); sin(Xe(3))];
    Xe(3) = Xe(3) + dt * vw(2);

    % Covarience matrix : P
    Jx = [1,0,-dt*vw(1)*sin(Xe(3)); 0,1,dt*vw(1)*cos(Xe(3)); 0,0,1];

    Ju = [dt*cos(Xe(3)),0; dt*sin(Xe(3)),0; 0,dt];
    Pu = Ju*Pu*Ju';

    P = Jx*P*Jx'+Pu;


end
%......................................................................



%......................................................................
% 寻找 每个OOI 距离最近的 landmark
function [Useful_OOIs, Useful_Landmarks] = GetUseful(OOIs, landmarks)

    for i = 1:size(OOIs, 2)
        
        currentOOI = OOIs(:,i);
        d = sum( (landmarks - currentOOI).^2, 1 );
        
        [distance, index] = min(d);
        
        distances(i) = distance;
        indexes(i) = index;

    end

    
    Useful_OOIs = OOIs( : , distances<1 );

    Useful_Landmarks = landmarks( : , [ indexes(:,distances<1) ] );

end
%......................................................................



%............................................................................................
% 获取当前的 H矩阵 
function H = GetJacobianH(Xe,Ulandmark)

    H = zeros(1,3);
    H(1) = -(Ulandmark(1)-Xe(1)) / sqrt( (Ulandmark(1)-Xe(1))^2 + (Ulandmark(2)-Xe(2))^2 );
    H(2) = -(Ulandmark(2)-Xe(2)) / sqrt( (Ulandmark(1)-Xe(1))^2 + (Ulandmark(2)-Xe(2))^2 );
    H(3) = 0;

end
%............................................................................................



%............................................................................................
% EKF Update
function [Xe,P] = EKFUpDate(Xe,P,ULandmark_GCF,r_measure,R)

    for i=1:size(ULandmark_GCF,2)
                            
        ULandmark = ULandmark_GCF(:,i);
        range = sqrt( (ULandmark(1)-Xe(1))^2 + (ULandmark(2)-Xe(2))^2 );
        
        z = r_measure(i)-range;
    
        H = GetJacobianH(Xe,ULandmark);
        S = H*P*H'+R;
        Si = inv(S);
        K = P*H'*Si;
    
        Xe = Xe + K*z;
        P = P- K*H*P;

        % 只使用 1个OOI 进行 Update
        % break;
    
    end

end
%............................................................................................
