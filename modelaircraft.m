function modelaircraft(lf,wf,D,space,c,b,ch,bh,cv,bv)
hold on
%plot fuselage
   voxel([0 -wf/2 0],[lf wf D],'r',1);
   axis([-5 10 -5 10 -5 10]);
%plot wings
  voxel([lf-space (-b/2) D/4], [c b D/2], 'g', 0.7);
%plot horizontal tail
    voxel([0 (-bh/2) D/4], [ch bh D/2], 'g', 0.7);
%plot vertical tail
    voxel([0 (-wf/4) D], [cv wf/2 bv], 'g', 0.7);
    hold off
end