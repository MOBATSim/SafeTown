classdef detectorSim < matlab.System & handle & matlab.system.mixin.Propagates
    % Detector used with a MATLAB System Block on Simulink.
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties   
        numberOfVehicles

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        detector
        vidPlayer
        intersectRects
       

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            trainedDetector = load('DetectorNewACF.mat');
            obj.detector = trainedDetector.detector;
            obj.vidPlayer = vision.DeployableVideoPlayer;
            
            intersectionRect1 = [860 410 140 230]; % Intersection 1 right
            intersectionRect2 = [295 420 140 230]; % Intersection 2 left
            intersectionRect3 = [530 60 210 140]; % Intersection 3 up
            intersectionRect4 = [540 420 190 230]; % Intersection 4 center
            intersectionRect5 = [540 885 210 125]; % Intersection 5 down

            obj.intersectRects = [intersectionRect1; intersectionRect2; 
            intersectionRect3;intersectionRect4;intersectionRect5]; % combine
        end

        function [y,bboxes] = stepImpl(obj,I)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            [bboxes, scores] = detect(obj.detector,I);
            
            [bboxes, scores] = selectStrongestBbox(bboxes,scores, 'OverlapThreshold', 0);
            
            % Show according to the number of vehicles
            if size(bboxes,1)>obj.numberOfVehicles
                % remove extra rectangles
                [~,idx]= sort(scores);
                bboxes = bboxes(idx(end-(obj.numberOfVehicles-1):end),:);
                scores = scores(idx(end-(obj.numberOfVehicles-1):end),:);
            else
                % do nothing
            end
            
            % Visualize
            for m = 1:length(scores)
                %annotation = sprintf('%s , Confidence %4.2f',detector.ModelName,scores(m));
                %Normalize scores
                scores = round(scores);
                scores(scores>100)=100;
                labelStr=strcat('Robot',{' '},num2str(scores(m)),'%');
                I = insertObjectAnnotation(I,'rectangle',bboxes(m,:),labelStr,'LineWidth',3,'TextBoxOpacity',0.9,'FontSize',18);
                

            I=obj.detectIntersection(I,bboxes,obj.intersectRects);
            end
            y = I;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function I=detectIntersection(obj,I,bboxes,intersectionRect)
            
            overlaps = rectint(bboxes,intersectionRect);
            % columns depict intersections in the overlap matrix
            
            [~,intersectNum]=size(overlaps);
            % get the number of intersections
            
            for k = 1:intersectNum
                % loop all the intersections
                if nnz(overlaps(:,k))>0
                    % if occupied color red
                    I = insertObjectAnnotation(I,'rectangle',intersectionRect(k,:),'Intersection','Color','red','LineWidth',3,'TextBoxOpacity',0.2,'FontSize',18);
                else
                    % if not occupied color green
                    I = insertObjectAnnotation(I,'rectangle',intersectionRect(k,:),'Intersection','Color','green','LineWidth',3,'TextBoxOpacity',0.2,'FontSize',18);
                end
            end
            
            
        end

        function [out,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1024 1280 3];
            out2 = [4 4];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "uint8";
            out2 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out,out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out,out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            out2 = false;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
