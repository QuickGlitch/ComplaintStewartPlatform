kBot = 1/7;
kTop = 6;
kDens = 5;
kRange = linspace(kBot,kTop,kDens)';
%kSpacePt1 = [ones(kDens,1),kRange, ones(kDens,1);];

kSpacePt2 = zeros(kDens*(kDens-1),3);
for iii = 1:kDens
    kSpacePt2(kDens*iii-(kDens-1):kDens*iii,:) = ...
        [kRange,ones(kDens,1),circshift(kRange,iii)];
end

kSpace = [];
for i = 1:length(kSpacePt2)
    if kSpacePt2(i,1)>kSpacePt2(i,3)
    kSpace = [kSpace; kSpacePt2(i,:)];
    end
end

