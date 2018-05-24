classdef Box
    properties
        x;
        y;
        width;
        height;
       
    end
    
    methods
        function obj = Box(myX,myY,myWidth,myHeight)
            obj.x = int32(myX);
            obj.y= int32(myY);
            obj.width = int32(myWidth);
            obj.height= int32(myHeight);
        end
            
        function [xOut,yOut,widthOut,heightOut] = data(this)
         
            xOut = int16(this.x);
            yOut = int16(this.y);
            widthOut = int16(this.width);
            heightOut = int16(this.height);
            
            end
         function range = getRange(this,number)
             number = lower(number);
             if strcmp(number,'x')
                 range = this.x : this.x + this.width;
             elseif strcmp(number,'y')
                 range = this.y : this.y + this.height;
             end
         end
    
          
        %{
        function enclosed = inBounds(myX,myY)
            enclosed = ((myX > Box.left() && myX< Box.right()) || (myY > Box.top() && myY < Box.bottom()));
        end
        function crosses = intersects(x,y)
            mat = int16(origImg(bounds.x + 1:bounds.x + bounds.width,bounds.y +1 :bounds.y + bounds.height) > 0);
        topInter =  sum((mat(1,:) ~= origImg(bounds.x + 1,1:size(mat,2))))
        leftInter = sum((mat(:,1) ~= origImg(1:size(mat,1), bounds.y + 1)))
        botInter =sum((mat(end,:) ~= origImg(bounds.x + 1 + bounds.width,1:size(mat,2))))
        rightInter = sum((mat(:,end) ~= origImg(1:size(mat,1), bounds.y + 1 + bounds.height)))
        crosses = (sum(sum(mat))> 0 & rightInter == 0 & leftInter ==0 & botInter == 0 & topInter ==0  )
        end
        %}
       
       
    end
end