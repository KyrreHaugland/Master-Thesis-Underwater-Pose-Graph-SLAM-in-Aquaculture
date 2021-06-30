%Note: Changes have been made to this script to better fit the camera data
%obtained from the fish cage survailance

function CLAHSImage = clahs(Image,NrY,NrX,varargin)
%CLAHS Contrast Limited Adaptive Histogram Specification
%     CLAHS locally enhances the contrast of images by 
%     transforming the values of each subregion, so that the 
%     histogram of the output subregion approximately matches a
%     specified histogram.  Bilinear interpolation is performed
%     across neighboring subregions to eliminate artifically
%     induced boundaries.
%
%     J = CLAHS(I,NRY,NRX) subdivides the image into NRY*NRX
%     subregions.  Each subregion is then contrast limited
%     histogram equalized so that the histogram of each output
%     subregion approximately matches a Rayleigh distribution
%     with parameter alpha = 0.4.  The default histogram bin 
%     size and the default number of grey levels are each 256
%     for a unit8 image and 65536 for a unit16 image.
%     A normalized contrast clip limit of 5 is default.  The
%     clipped pixels are uniformly distributed over the clipped 
%     histogram.
%
%     The I,NRY,NRX triple can be followed by parameter or
%     parameter/value pairs to specify additional arguments.
%     The following parameters, parameter/value arguments may
%     be specified:
%     Paramter/Value           Comment
%     -------------------------------------------------------
%     'Uniform'                Uniform distribution.
%     'Exponential',value      Exponential with parameter 0 < alpha < 1
%     'Rayleigh',value         Ralyleigh with parameter 0 <alpha < 1
%     'NrBins',value           Number of greybins for histogram
%     'ClipLimit',value        Normalized clip limit
%                              (higher values = more contrast)
%                              value <= 1 = standard AHE
%     'NrGreyLevels',value     Number of greylevel values
%     'Simple'                 Simply clip histogram without 
%                              redistributing clipped pixels
%     'Redistribute'           Clip histogram and redistribute
%                              clipped pixels uniformly over histogram
%     'PreserveMaxMin'         Preserves the original image intensity
%                              min max bounds.
%
%     Class Support
%     -------------
%     For syntaxes that include an intensity image I as input, I
%     can be of class uint8, uint16, or double, and the output 
%     image J has the same class as I.
% 
%     Example
%     -------
%     Enhance the contrast of an intensity image using contrast
%     limited adaptive histogram equalization. 
%  
%         I = imread('tire.tif');
%         J = clahs(I,5,4);
%         imshow(I), figure, imshow(J)

% History
% Who        Date         Comment
%--------    --------     ----------------------------
% rme        09/01        Significantly modifiy/rewrite to
%                         optimize originally implementation for
%                         matlab
% rme        03/02        Fix some bugs.

%================================
% initalize some default values
%================================
I = double(Image);
[YRes,XRes] = size(I);

%Setting default settings;
Min = 0;
Max = 255;
NrBins       = 256; % number of greybins for histogram ("dynamic range")
NrGreyLevels = 256; % number of greylevel values
ClipLimit    = 6;   % normalized cliplimit (higher values give more contrast)     
if isa(Image,'uint8')
    Min = 0;
    Max = 255;
    NrBins       = 256; % number of greybins for histogram ("dynamic range")
    NrGreyLevels = 256; % number of greylevel values
    ClipLimit    = 5;   % normalized cliplimit (higher values give more contrast)     
elseif isa(Image,'uint16')
    Min = 0;
    Max = 65535;
    NrBins       = 65536; % number of greybins for histogram ("dynamic range")
    NrGreyLevels = 65536; % number of greylevel values   
    ClipLimit    = 5*256; % normalized cliplimit (higher values give more contrast)
                          % scaled by 256 to compensate for expanded dynamic range
end
heq_type     = 3;   % 1 = uniform, 2 = exponential, 3 = rayleigh 
clip_type    = 1;   % 0 = no clipping, 1 = simple clip, 2 = clip and redistibute
alpha        = 0.4; % parameter used in exponential and rayleigh histogram specification
MAX_REG_X             = 128; % max # contextual regions in x-direction
MAX_REG_Y             = 128; % max # contextual regions in y-direction
HEQ_CLIP_SIMPLE       = 1;
HEQ_CLIP_REDISTRIBUTE = 2;
% The number of "effective greylevels in the output image is set by NrBins.
% The output image will have the same minimum and maximum value as the input
% image.  A clip limit smaller than 1 results in standard (non-contrast limited) AHE

%===============================================
% check for optional arguments
%===============================================
NARG= nargin;
if NARG < 3
    error('Invalid number of arguments');
elseif NARG > 3
    done = 0;
    ii = 1;
    while ~done
        switch lower(varargin{ii})
        case 'uniform';     heq_type = 1; ii = ii+1;
        case 'exponential'; heq_type = 2; alpha = varargin{ii+1}; ii = ii+2;
        case 'rayleigh';    heq_type = 3; alpha = varargin{ii+1}; ii = ii+2;
        case 'nrbins';      NrBins = varargin{ii+1}; ii=ii+2;
        case 'cliplimit';   ClipLimit = varargin{ii+1}; ii=ii+2;
        case 'nrgreylevels';NrGreyLevels = varargin{ii+1}; ii=ii+2;
        case 'simple';      clip_type = HEQ_CLIP_SIMPLE; ii=ii+1;
        case 'redistribute';clip_type = HEQ_CLIP_REDISTRIBUTE; ii=ii+1;
        case 'preserveminmax'; Min = min(I(:)); Max = max(I(:)); ii=ii+1;
        otherwise
            msg = sprintf('Unknown argument given');
            error(msg);
        end
        if ii >= length(varargin); done = 1; end
    end
end


status = 0; 
%======================================
% various sanity checks on the inputs
%======================================
if NrX > MAX_REG_X
    msg = sprintf('NrX > MAX_REG_X = %g',MAX_REG_X);
    error(msg);
end
if NrY > MAX_REG_Y
    msg = sprintf('NrY > MAX_REG_Y = %g',MAX_REG_Y);
    error(msg);
end
if mod(XRes,NrX)
    msg = sprintf('XRes/NrX = %g/%g = %g, noninteger value', ...
        XRes,NrX,XRes/NrX);
    error(msg);
end
if mod(YRes,NrY)
    msg = sprintf('YRes/NrY = %g/%g = %g, noninteger value', ...
        YRes,NrY,YRes/NrY);
    error(msg);
end
if Max > NrGreyLevels
    msg = sprintf('Max image value %g > number of greylevels %g', ...
        Max,NrGreyLevels);
    error(msg);
end
if (NrX <2 | NrY <2)
    msg = sprintf('NrX < 2 or NrY < 2, number of subregions must be >= 4');
    error(msg);
end
if ClipLimit <= 1
    msg = sprintf('ClipLimit <= 1, performing standard AHE ...');
    disp(msg);
elseif isa(Image,'uint16')
    ClipLimit = ClipLimit*256;
end

%==============================
% setup/initalization
%==============================
% create an array to hold the histogram for each subregion
MapArray = zeros(NrX*NrY*NrBins,1);

% determine properties of each subregion
XSize = XRes/NrX;
YSize = YRes/NrY;
NrPixels = XSize*YSize;

% calculate actual clip limit
if ClipLimit > 1
    ClipLimit = (ClipLimit*NrPixels/NrBins);
    if ClipLimit < 1
        ClipLimit = 1;
    else
        ClipLimit = ClipLimit;
    end 
else
    % large value, do not clip (AHE)    
    ClipLimit = inf;
    clip_type = 0;
end

%==================================================
% calculate greylevel mappings for each subregion
%==================================================
% calculate and process histograms for each subregion
for Y = 0:(NrY-1)
    for X = 0:(NrX-1)
        Xtmp = X*XSize+1;
        Ytmp = Y*YSize+1;
        indX = Xtmp:Xtmp+XSize-1;
        indY = Ytmp:Ytmp+YSize-1;
        SubRegion = I(indY,indX);
        if NrBins >= NrGreyLevels
            Hist = histc(SubRegion(:),[0:NrBins-1]);
        else
            Hist = histc(SubRegion(:),linspace(0,NrGreyLevels,NrBins));
        end
        %figure(1); bar(Hist);
        
        % clip histogram, simple or redistribute depending on input parameters 	
        if clip_type == HEQ_CLIP_REDISTRIBUTE
            Hist = ClipHistogram(Hist,NrBins,ClipLimit);
        elseif clip_type == HEQ_CLIP_SIMPLE
            Hist  = ClipHistogramSimple(Hist,NrBins,ClipLimit);
        end

        %figure(2);bar(Hist);
        % create histogram mapping (uniform,exponential,or rayleigh)     
        Hist = MapHistogram(Hist, Min, Max, NrBins, NrPixels, heq_type, alpha);
        %figure(3);bar(Hist); pause;
        % write working histogram into appropriate part of MapArray
        MapArray((NrBins *(Y*NrX+X))+1:(NrBins *(Y*NrX+X))+NrBins) = Hist;
    end
end

%====================================================
% interpolate greylevel mappings to get CLAHE image
%====================================================
% make lookup table for mapping of grey values
LUT = zeros(NrGreyLevels,1);
LUT = MakeLUT(LUT, Min, Max, NrBins);
CLAHSImage = zeros(YRes,XRes);
lenY = 1;
for Y = 0:NrY
    if Y == 0       %special case top row
        SubY = floor(YSize/2); YU = 0; YB = 0;
    elseif Y == NrY %special case bottom row 
        SubY = floor(YSize/2); YU = NrY-1; YB = YU;
    else
        SubY = YSize; YU = Y-1; YB = YU+1;
    end
    indY = lenY:lenY+SubY-1;
    lenX = 1;
    for X = 0:NrX
        if X==0         %special case Left column
            SubX = floor(XSize/2); XL = 0; XR = 0;
        elseif X == NrX %special case right column
            SubX = floor(XSize/2); XL = NrX-1; XR = XL;
        else 
            SubX = XSize; XL = X-1; XR = XL+1;
        end
        
        % retrieve the appropriate histogram mappings from MapArray	
        LU = MapArray((NrBins*(YU*NrX +XL))+1:((NrBins*(YU*NrX +XL)))+NrBins);  
        RU = MapArray((NrBins*(YU*NrX +XR))+1:((NrBins*(YU*NrX +XR)))+NrBins);
        LB = MapArray((NrBins*(YB*NrX +XL))+1:((NrBins*(YB*NrX +XL)))+NrBins);
        RB = MapArray((NrBins*(YB*NrX +XR))+1:((NrBins*(YB*NrX +XR)))+NrBins);
        
        % interpolate the appropriate subregion
        indX = lenX:lenX+SubX-1;
        SubRegion = I(indY,indX);
        InterpD = Interpolate(SubRegion,LU,RU,LB,RB,SubX,SubY,LUT);
        CLAHSImage(indY,indX) = InterpD;
        lenX = lenX+SubX;
    end
    lenY = lenY+SubY;
end

%=============================================================
% output the CLAHEImage in the same format as the input image
%=============================================================
if isa(Image,'uint16'); CLAHSImage = uint16(CLAHSImage);
else; CLAHSImage = uint8(CLAHSImage);
end
%=====================================================================================



%==================================
% Private Subfunctions
%==================================

%=====================================================================================
function LUT = MakeLUT(LUT,Min,Max,NrBins)
% To speed up histogram clipping, the input image [Min,Max] is scaled down to
% [0,NrBins-1].  This function calculates the LUT.
BinSize = 1 + floor((Max-Min)/NrBins);
i = (Min+1):(Max+1);
LUT(i) = min(1 + floor((i - Min)/BinSize),NrBins);
%===================================================================================== 

%=====================================================================================
function NewHistogram  = ClipHistogram(Histogram,NrGreyLevels,ClipLimit)
% This function performs clipping of the histogram and redistribution of the 
% clipped bin elements.  Histogram is clipped and excess elements are counted
% Then excess elements are equally reditributed over the whole histogram
% (providing the bin count is smaller than the cliplimit)

% number of excess pixels created by clipping
NrExcess = sum(max(Histogram-ClipLimit,0));

%====================================================================
% clip Histogram and redistribute excess bin elements in each bin
%====================================================================
% # of elements to be redist'ed to each bin  
BinIncr	= floor(NrExcess/NrGreyLevels);
% max bin value where redist. will be above Climit
Upper = ClipLimit - BinIncr; 

% clip the histogram to ClipLimit
Histogram = min(Histogram,ClipLimit);
% add partial BinIncr pixels to bins up to ClipLimit
ii = find(Histogram > Upper);
NrExcess = NrExcess - sum(ClipLimit - Histogram(ii));
Histogram(ii) = ClipLimit;
% add BinIncr to all other bins
jj = find(Histogram <= Upper);
NrExcess = NrExcess - length(jj)*BinIncr;
Histogram(jj) = Histogram(jj) + BinIncr;

% evenly redistribute remaining excess pixels
while NrExcess>0
    h = 1;
    while (h < NrGreyLevels && NrExcess > 0) 
        % choose step to distribute the most excess evenly in one pass 
        StepSize = ceil(NrGreyLevels/NrExcess);
        i = h;
        while (i<(NrGreyLevels+1) && NrExcess >0), 
            if Histogram(i) < ClipLimit
                Histogram(i) =  Histogram(i) + 1;
                NrExcess = NrExcess - 1;
            end
            i = i + StepSize; % step along the histogram
        end
        h = h + 1; % avoid concentrating pixels in bin 1
    end
end
NewHistogram = Histogram;
%=====================================================================================

%=====================================================================================
function NewHistogram  = ClipHistogramSimple(Histogram,NrGreyLevels,ClipLimit)
% This function performs clipping of the histogram
% any bin with a value above the cliplimit is assigned the value of ClipLimit

NewHistogram = min(Histogram,ClipLimit);
%=====================================================================================


%=====================================================================================
function Histogram = MapHistogram(Histogram, Min, Max, NrGreyLevels, NrofPixels, heq_type, heq_alpha)
% This function calculates the equalized lookup table (mapping)
% by cumulating the input histogram
% Note: Lookup table is rescaled in the range [Min..Max].

HEQ_NONE        = 0;
HEQ_UNIFORM     = 1; 
HEQ_EXPONENTIAL = 2;
HEQ_RAYLEIGH    = 3;

Scale = (Max-Min)/NrofPixels;

switch heq_type    
case HEQ_UNIFORM,  %accummulate histogram uniformly
    Sum = cumsum(Histogram);
    % scale it from min to max
    Histogram = min(Min + Sum*Scale, Max); %limit range to Max
case HEQ_EXPONENTIAL, %accumulate histogram exponentially 
    Sum = cumsum(Histogram);
    vmax = 1.0 - exp(-heq_alpha);
    temp = -1/heq_alpha*log(1-(vmax*Sum/NrofPixels));
    Histogram = min(temp*Max,Max); %limit range to Max
case HEQ_RAYLEIGH, %accumulate histogram using rayleigh
    Sum = cumsum(Histogram);
    hconst = 2*heq_alpha*heq_alpha;
    vmax = 1 - exp((-1/hconst));
    temp = sqrt(-hconst*log(1-vmax*(double(Sum)/NrofPixels)));
    Histogram = min(temp*Max,Max); %limit range to Max
otherwise, %just do UNIFORM if heq_type has a wacky value
    Sum = cumsum(Histogram);
    Histogram = min(Min + Sum*Scale,Max); %limit range to Max
end  
%=====================================================================================

%=====================================================================================
function InterpRegion = Interpolate(SubRegion,MapLU,MapRU,MapLB,MapRB,XSize,YSize,LUT)
% This function calculates the new greylevel assignment of pixels within
% a subregion of the image with size XSize YSize.  This is done by a bilinear
% interpolation between four different histogram mappings in order to eliminate
% boundary artifacts.

Num = XSize * YSize; %Normalization factor
BinValues = LUT(SubRegion+1);
[XInvCoef{1:YSize,1}] = deal([XSize:-1:1]);  XInvCoef = cell2mat(XInvCoef);
[YInvCoef{1,1:XSize}] = deal([YSize:-1:1]'); YInvCoef = cell2mat(YInvCoef);
[XCoef{1:YSize,1}] = deal([0:XSize-1]);      XCoef    = cell2mat(XCoef);
[YCoef{1,1:XSize}] = deal([0:YSize-1]');     YCoef    = cell2mat(YCoef);

InterpRegion = (YInvCoef.*(XInvCoef.*MapLU(BinValues) + XCoef.*MapRU(BinValues)) ...
    + YCoef.*(XInvCoef.*MapLB(BinValues) + XCoef.*MapRB(BinValues)))/Num;
%=====================================================================================