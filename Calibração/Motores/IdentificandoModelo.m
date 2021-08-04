clear all
clc
correnteMaxima = 4;
pwmA = 20:10:100;
correnteA = [0.12, 0.46, 0.73, 1.06, 1.42, 1.75, 2.30, 3.06, 3.90];
pA = polyfit(correnteA,pwmA,3)
plot(correnteA,pwmA,'o')
hold on
plot(correnteA,polyval(pA,correnteA),'g--')

pwmB = 20:10:100;
correnteB = [0.12, 0.56, 0.97, 1.45, 1.96, 2.88, 4.11, 5.28, 6.72];
pB = polyfit(correnteB,pwmB,3)
plot(correnteB,pwmB,'o')
hold on
plot(correnteB,polyval(pB,correnteB),'r--')
% yfit = p1(1) * x + p1(2);
% yresid = y - yfit;
% SQresid = sum(yresid.^2);
% SQtotal = (length(y)-1) * var(y);
% R2 = 1 - SQresid/SQtotal

pwmAMax=polyval(pA,correnteMaxima)
pwmBMax=polyval(pB,correnteMaxima)

porcentagem = 10:10:100;
correntesEsperadas= porcentagem*correnteMaxima/100
pwmAEsperado=polyval(pA,correntesEsperadas)
pwmBEsperado=polyval(pB,correntesEsperadas)
