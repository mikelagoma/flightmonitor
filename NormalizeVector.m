function [ mag uVector ] = NormalizeVector( pVector)
    mag = sqrt(sum(pVector.^2));
    if( mag == 0)
        uVector = pVector;
    else
        uVector = pVector/mag;
    end
end

