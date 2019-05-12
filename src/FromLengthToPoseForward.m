clear;
%% SixDof Platform Config
syms UpZ DownZ UpR DownR DisUp DisDown
AXIS_COUNT = 6;
CIRCLE_TOP_RADIUS = UpR;
CIRCLE_BOTTOM_RADIUS = DownR;
DIS_BETWEEN_UP_HINGE = DisUp;
DIS_BETWEEN_DOWN_HINGE = DisDown;
PLANE_ABOVE_HINGE_LENGTH = UpZ;
PLANE_ABOVE_BOTTOM_LENGTH = DownZ;
SIN_30_DEG = 0.5;
COS_30_DEG = 0.86602540378;
SIN_60_DEG = 0.86602540378;
COS_60_DEG = 0.5;
RAD_30 =  (30.0 / 180.0 * pi);
RAD_60 = (60.0 / 180.0 * pi);

%% AxisInit

HingePositions = {[0;0;-PLANE_ABOVE_HINGE_LENGTH] [0;0;-PLANE_ABOVE_HINGE_LENGTH] 
                  [0;0;-PLANE_ABOVE_HINGE_LENGTH] [0;0;-PLANE_ABOVE_HINGE_LENGTH] 
                  [0;0;-PLANE_ABOVE_HINGE_LENGTH] [0;0;-PLANE_ABOVE_HINGE_LENGTH]};
BottomPositions = {[0;0;-PLANE_ABOVE_BOTTOM_LENGTH] [0;0;-PLANE_ABOVE_BOTTOM_LENGTH] 
                   [0;0;-PLANE_ABOVE_BOTTOM_LENGTH] [0;0;-PLANE_ABOVE_BOTTOM_LENGTH] 
                   [0;0;-PLANE_ABOVE_BOTTOM_LENGTH] [0;0;-PLANE_ABOVE_BOTTOM_LENGTH]};

top_between_hinge_rad = CosineTheorem(DIS_BETWEEN_UP_HINGE, CIRCLE_TOP_RADIUS, CIRCLE_TOP_RADIUS) / 2.0;
bottom_between_hinge_rad = CosineTheorem(DIS_BETWEEN_DOWN_HINGE, CIRCLE_BOTTOM_RADIUS, CIRCLE_BOTTOM_RADIUS) / 2.0;
rad30 = RAD_30;
rad60 = RAD_60;

BottomPositions{1}(1) = CIRCLE_BOTTOM_RADIUS * cos(bottom_between_hinge_rad);
BottomPositions{1}(2) = CIRCLE_BOTTOM_RADIUS * sin(bottom_between_hinge_rad);

BottomPositions{2}(1) = -CIRCLE_BOTTOM_RADIUS * sin(rad30 - bottom_between_hinge_rad);
BottomPositions{2}(2) = CIRCLE_BOTTOM_RADIUS * cos(rad30 - bottom_between_hinge_rad);

BottomPositions{3}(1) = -CIRCLE_BOTTOM_RADIUS * sin(rad30 + bottom_between_hinge_rad);
BottomPositions{3}(2) = CIRCLE_BOTTOM_RADIUS * cos(rad30 + bottom_between_hinge_rad);

BottomPositions{4}(1) = BottomPositions{3}(1);
BottomPositions{4}(2) = -BottomPositions{3}(2);

BottomPositions{5}(1) = BottomPositions{2}(1);
BottomPositions{5}(2) = -BottomPositions{2}(2);

BottomPositions{6}(1) = BottomPositions{1}(1);
BottomPositions{6}(2) = -BottomPositions{1}(2);

HingePositions{1}(1) = CIRCLE_TOP_RADIUS * cos(rad60 - top_between_hinge_rad);
HingePositions{1}(2) = CIRCLE_TOP_RADIUS * sin(rad60 - top_between_hinge_rad);

HingePositions{2}(1) = CIRCLE_TOP_RADIUS * cos(rad60 + top_between_hinge_rad);
HingePositions{2}(2) = CIRCLE_TOP_RADIUS * sin(rad60 + top_between_hinge_rad);

HingePositions{3}(1) = -CIRCLE_TOP_RADIUS * cos(top_between_hinge_rad);
HingePositions{3}(2) = CIRCLE_TOP_RADIUS * sin(top_between_hinge_rad);

HingePositions{4}(1) = HingePositions{3}(1);
HingePositions{4}(2) = -HingePositions{3}(2);

HingePositions{5}(1) = HingePositions{2}(1);
HingePositions{5}(2) = -HingePositions{2}(2);

HingePositions{6}(1) = HingePositions{1}(1);
HingePositions{6}(2) = -HingePositions{1}(2);

AxisInitLength = {[], [], [], [], [], []};
syms dLen1 dLen2 dLen3 dLen4 dLen5 dLen6
AxisDeltaLength = [dLen1 dLen2 dLen3 dLen4 dLen5 dLen6];

for i = 1: AXIS_COUNT 
    v = (HingePositions{i} - BottomPositions{i}) .^ 2;
    length = sqrt(sum(v));
    AxisInitLength{i} = length;
end

%% Forward Control
syms x y z a b c
R = BuildConversionMatrix(c, a, b);
ZeroPoint = [0; 0; 0];
deltaPoint = [x; y; z];
f = {[], [], [], [], [], []};
for i = 1: AXIS_COUNT 
    positionU = HingePositions{i};
    deltaPoint = [x; y; z];
    positionUforO = R * positionU;
    nowPosition = ZeroPoint + deltaPoint + positionUforO - BottomPositions{i};
    f{i} = sqrt(sum(nowPosition.^2)) - AxisInitLength(i) - AxisDeltaLength(i);
    dfdx = diff(f{i}, x)
    dfdy = diff(f{i}, y)
    dfdz = diff(f{i}, z)
    dfda = diff(f{i}, a)
    dfdb = diff(f{i}, b)
    dfdc = diff(f{i}, c)
end
f1 = f{1}
f2 = f{2}
f3 = f{3}
f4 = f{4}
f5 = f{5}
f6 = f{6}







