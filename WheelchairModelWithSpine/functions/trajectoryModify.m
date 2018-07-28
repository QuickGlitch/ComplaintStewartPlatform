temp = Kinematics.EM - Kinematics.US;
USEMdistanceRAW = sqrt(temp(:,1).^2 + temp(:,2).^2 + temp(:,3).^2);
USEMratios = temp./repmat(USEMdistanceRAW,[1,3]);

DesiredL = 328.3295;

Deviations = DesiredL - USEMdistanceRAW;
Adjustments = USEMratios .* repmat(Deviations,[1 3]);
EMmodified = USEMratios*DesiredL + Kinematics.US; %Kinematics.EM + Adjustments;



