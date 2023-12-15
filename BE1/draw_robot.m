function draw_robot(x,color,sizerobot)
   if (exist('sizerobot')==0), sizerobot=1; end;
   M = sizerobot*...
      [  1 -1  0  0 -1 -1 0 0 -1 1 0 0 3  3  0; 
	    -2 -2 -2 -1 -1  1 1 2  2 2 2 1 0.5 -0.5 -1];
   M=[M;ones(1,length(M))]; 
   R=[cos(x(3)),-sin(x(3)),x(1);sin(x(3)),cos(x(3)),x(2);0 0 1];    
   M =R*M;
   plot(M(1,:),M(2,:),color,'LineWidth',2);       
end

