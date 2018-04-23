classdef MatrixStack < handle
    
    properties (Access = private)
        stack
        depth
    end
    
    methods
        
        function obj = MatrixStack()
            obj.stack = zeros(4, 4, 10);
            obj.depth = 1;
            
            obj.setIdentity();
        end
        
        function push(obj)
            obj.stack(:,:,obj.depth+1) = obj.stack(:,:,obj.depth);
            obj.depth = obj.depth + 1;
        end
        
        function pop(obj)
            if obj.depth == 1
                error('Stack underflow');
            end
            obj.depth = obj.depth - 1;
        end
        
        function m = top(obj)
            m = obj.stack(:,:,obj.depth);
        end
        
        function translate(obj, x, y, z)
            t = [1 0 0 x;
                 0 1 0 y;
                 0 0 1 z;
                 0 0 0 1];
            
            obj.stack(:,:,obj.depth) = obj.stack(:,:,obj.depth) * t; 
        end
        
        function rotate(obj, angle, x, y, z)
            c = cosd(angle);
            s = sind(angle);
            r = [  x^2*(1-c)+c x*y*(1-c)-z*s x*z*(1-c)+y*s 0;
                 y*x*(1-c)+z*s   y^2*(1-c)+c y*z*(1-c)-x*s 0;
                 x*z*(1-c)-y*s y*z*(1-c)+x*s   z^2*(1-c)+c 0;
                             0             0             0 1];
                         
            obj.stack(:,:,obj.depth) = obj.stack(:,:,obj.depth) * r;
        end
        
        function scale(obj, x, y, z)
            s = [x 0 0 0;
                 0 y 0 0;
                 0 0 z 0;
                 0 0 0 1];
             
             obj.stack(:,:,obj.depth) = obj.stack(:,:,obj.depth) * s;
        end
        
        function orthographic(obj, left, right, bottom, top, zNear, zFar)
            if nargin < 6
                zNear = -1;
            end
            if nargin < 7
                zFar = 1;
            end
            
            tx = -(right+left)/(right-left);
            ty = -(top+bottom)/(top-bottom);
            tz = -(zFar+zNear)/(zFar-zNear);
            o = [2/(right-left)              0               0 tx;
                              0 2/(top-bottom)               0 ty;
                              0              0 -2/(zFar-zNear) tz;
                              0              0               0  1];
                          
            obj.stack(:,:,obj.depth) = obj.stack(:,:,obj.depth) * o;
        end
        
        function perspective(obj, fovy, aspect, zNear, zFar)
            if nargin < 4
                zNear = 0.1;
            end
            if nargin < 5
                zFar = 10000;
            end
            
            rad = deg2rad(fovy);
            f = cot(rad/2);
            p = [f/aspect 0                          0                          0;
                        0 f                          0                          0;
                        0 0 -(zFar+zNear)/(zFar-zNear) -2*zFar*zNear/(zFar-zNear);
                        0 0                         -1                          0];
                    
            obj.stack(:,:,obj.depth) = obj.stack(:,:,obj.depth) * p;
        end
        
        function flyPerspective(obj,screenDim)
            w = screenDim(1); h = screenDim(2);
            pa = [-w/2, -h/2, w/2]; % lower left
            pb = [ w/2, -h/2, w/2]; % lower right
            pc = [-w/2, h/2, w/2]; % upper left

            % determine screen unit vectors
            vr = (pb - pa) ./ norm(pb - pa);
            vu = (pc - pa) ./ norm(pc - pa);

            vn = cross(vr,vu);
            vn = vn ./ norm(vn);

            % Rotation matrix
            R = [vr(1), vu(1), vn(1), 0;
                 vr(2), vu(2), vn(2), 0;
                 vr(3), vu(3), vn(3), 0;
                 0, 0, 0, 1];

            pe = [0, 0, 0]; %fly location
            n=0.1; %near
            f=10000; %far

            % Determine frustum extents
            va = pa - pe;
            vb = pb - pe;
            vc = pc - pe;

            % Determine distance to screen
            d = -dot(vn, va);

            % Compute screen coordinates
            l = dot(vr, va) * n/d;
            r = dot(vr, vb) * n/d;
            b = dot(vu, va) * n/d;
            t = dot(vu, vc) * n/d;

            % Projection matrix
            P = [2*n/(r-l), 0, (r+l)/(r-l), 0;
                 0, (2.0*n)/(t-b), (t+b)/(t-b), 0;
                 0, 0, -(f+n)/(f-n), -(2.0*f*n)/(f-n);
                 0, 0, -1, 0];

            % Translation matrix
            T = [1, 0, 0, -pe(1);
                 0, 1, 0, -pe(2);
                 0, 0, 1, -pe(3);
                 0, 0, 0, 1];
            
            
            projMatrix = P * R * T;
                    
            obj.stack(:,:,obj.depth) = obj.stack(:,:,obj.depth) * projMatrix;
        end
        
        function setMatrix(obj, m)
            obj.stack(:,:,obj.depth) = m;
        end
        
        function setIdentity(obj)
            obj.stack(:,:,obj.depth) = [1 0 0 0;
                                        0 1 0 0;
                                        0 0 1 0;
                                        0 0 0 1];
        end
        
    end
    
end