classdef Canvas < handle
    
    properties (SetAccess = private)
        window      % Window containing the canvas
        size        % Size of the canvas [width, height] (pixels)
        projection  % Projection matrix stack
        modelView   % Model/View matrix stack
    end
    
    properties (Access = private)
        standardPrograms
        currentProgram
        defaultMask
        
        projectionUniform
        modelViewUniform
        colorUniform
        
        windowBeingDestroyed
    end
    
    methods
        
        function obj = Canvas(window)
            obj.window = window;
            obj.windowBeingDestroyed = addlistener(window, 'ObjectBeingDestroyed', @(e,d)obj.delete());
            
            obj.projection = MatrixStack();
            obj.projection.orthographic(0, window.size(1), 0, window.size(2));
            obj.modelView = MatrixStack();
            
            obj.standardPrograms = StandardPrograms(obj);
            
            obj.defaultMask = TextureObject(obj, 2);
            obj.defaultMask.setImage(ones(1, 1, 4, 'uint8') * 255);
            
            obj.resetBlend();
        end
        
        function s = get.size(obj)
            s = obj.window.size;
        end
        
        function makeCurrent(obj)
            glfwMakeContextCurrent(obj.window.handle);
        end
        
        function setClearColor(obj, color)
            obj.makeCurrent();
            
            c = color;
            if length(c) == 1
                c = [c, c, c, 1];
            elseif length(c) == 3
                c = [c, 1];
            end
            glClearColor(c(1), c(2), c(3), c(4));
        end
        
        function clear(obj)
            obj.makeCurrent();
            glClear(GL.COLOR_BUFFER_BIT);
        end
        
        function setProgram(obj, programName)            
            switch programName
                case 'PositionOnly'
                    program = obj.standardPrograms.positionOnlyProgram;
                case 'SingleTexture'
                    program = obj.standardPrograms.singleTextureProgram;
                otherwise
                    error('Unknown program name');
            end
            
            if program == obj.currentProgram
                return;
            end
            
            obj.makeCurrent();
            
            obj.projectionUniform = program.getUniformLocation('projectionMatrix');
            obj.modelViewUniform = program.getUniformLocation('modelViewMatrix');
            obj.colorUniform = program.getUniformLocation('color0');
            
            glUseProgram(program.handle);
            obj.currentProgram = program;
        end
        
        function enableBlend(obj, src, dest)            
            obj.makeCurrent();
            glEnable(GL.BLEND);
            glBlendFunc(src, dest);
        end
        
        function disableBlend(obj)
            obj.makeCurrent();
            glDisable(GL.BLEND);
        end
        
        function resetBlend(obj)
            obj.enableBlend(GL.SRC_ALPHA, GL.ONE_MINUS_SRC_ALPHA);
        end
        
        % Gets image matrix of current framebuffer data. 
        function d = getPixelData(obj, mode)
            if nargin < 2
                mode = GL.FRONT;
            end
            
            obj.makeCurrent();
            glReadBuffer(mode);
            d = glReadPixels(0, 0, obj.size(1), obj.size(2), GL.RGB, GL.UNSIGNED_BYTE);
            d = imrotate(d, 90);
        end
        
        function drawArray(obj, array, mode, first, count, color, texture, mask)
            obj.makeCurrent();
            
            if nargin < 7
                obj.setProgram('PositionOnly');
            else
                obj.setProgram('SingleTexture');
                
                glActiveTexture(GL.TEXTURE0);
                glBindTexture(texture.target, texture.handle);
                
                glActiveTexture(GL.TEXTURE1);
                if nargin >= 8
                    glBindTexture(mask.texture.target, mask.texture.handle);
                else
                    glBindTexture(obj.defaultMask.target, obj.defaultMask.handle);
                end
            end
            
            prog = obj.currentProgram;
            prog.setUniformMatrix(obj.projectionUniform, obj.projection.top());
            prog.setUniformMatrix(obj.modelViewUniform, obj.modelView.top());
            prog.setUniformfv(obj.colorUniform, color);
            
            glBindVertexArray(array.handle);
            glDrawArrays(mode, first, count);
            glBindVertexArray(0);
            
            if nargin > 6
                glBindTexture(texture.target, 0);
            end
        end
        
    end
    
end

