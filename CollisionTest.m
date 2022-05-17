clear
clc

%% Set up sign and robot
% set up robot
cobotLog = Log;
tempPose = transl(0,0,0);
dombot = MyCobot(cobotLog, 2, tempPose);

% set up sign - Pose doesnt work as envObj is not called 
signLog = Log;
tempPose2 = transl(1,0,0);
sign = StopSign(signLog, 2, tempPose2);

% get face and vertex data from the sign for intersection checks
face = sign.model.Faces;
vertex = sign.model.Vertices;

% determine the normal vectors for faces required for intercept checking
faceNormals = zeros(size(face,1),3);
    for faceIndex = 1:size(face,1)
        v1 = vertex(face(faceIndex,1)',:);
        v2 = vertex(face(faceIndex,2)',:);
        v3 = vertex(face(faceIndex,3)',:);
        % add to normals array
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end

% sign.UpdatePose(trotz(pi/2));
% 
% faceB = sign.model.Faces;
% vertexB = sign.model.Vertices;
% 
% faceNormalsB = zeros(size(faceB,1),3);
%     for faceIndex = 1:size(faceB,1)
%         v1 = vertexB(faceB(faceIndex,1)',:);
%         v2 = vertexB(faceB(faceIndex,2)',:);
%         v3 = vertexB(faceB(faceIndex,3)',:);
%         faceNormalsB(faceIndex,:) = unit(cross(v2-v1,v3-v1));
%     end

% [tr, allTR] = dombot.model.fkine(dombot.model.getpos);

% for i = 1:dombot.model.n
%     

%% LinePlaneIntersection - LAB 5
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

intersectionPoint = [0 0 0];
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;
D = dot(planeNormal,u);
N = -dot(planeNormal,w);
check = 0; %#ok<NASGU>
if abs(D) < 10^-7        % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check = 2;
        return
    else
        check = 0;       %no intersection
        return
    end
end

%compute the intersection parameter
sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

if (sI < 0 || sI > 1)
    check= 3;          %The intersection point  lies outside the segment, so there is no intersection
else
    check=1;
end
end

%% IsIntersectionPointInsideTriangle - LAB 5
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision - LAB 5
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

