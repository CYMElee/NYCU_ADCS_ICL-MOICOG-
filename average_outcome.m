% this .m file use to get the average value for Estimated M.O.I,P.O.I,C.o.G
Number_of_data = length(platform1.t(2:end));

MoI_average =zeros(3,1) ;
PoI_average =zeros(3,1) ;
CoG_average =zeros(3,1) ;

for i = 0:19
   MoI_average=MoI_average+[theta_array_platform1(1,Number_of_data-i);theta_array_platform1(2,Number_of_data-i);theta_array_platform1(3,Number_of_data-i)];
   PoI_average = PoI_average+[theta_array_platform1(4,Number_of_data-i);theta_array_platform1(5,Number_of_data-i);theta_array_platform1(6,Number_of_data-i)];
   CoG_average = CoG_average + [theta_array_platform1(7,Number_of_data-i);theta_array_platform1(8,Number_of_data-i);theta_array_platform1(9,Number_of_data-i)];
end


MoI_average = (MoI_average/20);
PoI_average = (PoI_average/20);
CoG_average = (CoG_average/20);