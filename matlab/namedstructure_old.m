% loads data into named structure
function out = namedstructure_old(A)
    out.time = normalizeTime( A.data(:,1) );
    out.pos_x = A.data(:,2);
    out.pos_y = A.data(:,3);
    out.pos_z = A.data(:,4);
    out.rot_x = A.data(:,5);
    out.rot_y = A.data(:,6);
    out.rot_z = A.data(:,7);
    out.rot_w = A.data(:,8);
    out.total_score=A.data(:,9);
    out.winning_margin=A.data(:,10);
    out.return_value_mean=A.data(:,11);
    out.return_value_stddev=A.data(:,12);
    out.total_cost=A.data(:,13);
    out.costsum=integrate(A.data(:,13));
    out.nrofunknownvoxels=A.data(:,14);
    out.averageuncertainty=A.data(:,15);
    out.averageendpointuncertainty=A.data(:,16);
    out.unknownobjectsidefrontier=A.data(:,17);
    out.unknownobjectvolumefrontier=A.data(:,18);
    out.classicfrontier=A.data(:,19);
    out.endnodeoccupancysum=A.data(:,20);
    out.totaloccupancycertainty=A.data(:,21);
    out.totalnrofoccupieds=A.data(:,22);
end