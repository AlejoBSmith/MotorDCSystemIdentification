clear
load datosmotor.mat
D0A0Ruido=Delay0Avg0solocalculoruido.rpm;
D0A10Ruido=Delay0Avg10.rpm;
D0A20Ruido=Delay0Avg20.rpm;
D0A30Ruido=Delay0Avg30.rpm;
%RV es de Referencia Variable
D0A0RV=Delay0Avg0refvariable.rpm;
D10A0RV=Delay10Avg0refvariable.rpm;
D20A0RV=Delay20Avg0refvariable.rpm;
D30A0RV=Delay30Avg0refvariable.rpm;
D10A10RVRPM=Delay10Avg10.rpm;
D10A10RVREF=Delay10Avg10.ref;
D20A20RVRPM=Delay20Avg20.rpm;
D20A20RVREF=Delay20Avg20.ref;
D30A30RVRPM=Delay30Avg30.rpm;
D30A30RVREF=Delay30Avg30.ref;
D10A10RVRPMPROP=Delay10Avg10Prop.rpm;
D10A10RVREFPROP=Delay10Avg10Prop.ref;
D20A20RVRPMPROP=Delay20Avg20Prop.rpm;
D20A20RVREFPROP=Delay20Avg20Prop.ref;
D30A30RVRPMPROP=Delay30Avg30Prop.rpm;
D30A30RVREFPROP=Delay30Avg30Prop.ref;
D50A50RVRPMPROP=Delay50Avg50Prop.rpm;
D50A50RVREFPROP=Delay50Avg50Prop.ref;
D100A100RVRPMPROP=Delay100Avg100Prop.rpm;
D100A100RVREFPROP=Delay100Avg100Prop.ref;

% Comparación aumento promediado señal
figure
subplot(2,2,1);
plot(D0A0Ruido);
title('Delay 0 Avg 0')
subplot(2,2,2);
plot(D0A10Ruido);
title('Delay 0 Avg 10')
subplot(2,2,3);
plot(D0A20Ruido);
title('Delay 0 Avg 20')
subplot(2,2,4);
plot(D0A30Ruido);
title('Delay 0 Avg 30')
sgtitle('Comparación aumento promediado RPM')

% Comparación aumento promediado señal
figure
subplot(2,2,1);
plot(D0A0RV);
title('Delay 0 Avg 0')
subplot(2,2,2);
plot(D10A0RV);
title('Delay 10 Avg 0')
subplot(2,2,3);
plot(D20A0RV);
title('Delay 20 Avg 0')
subplot(2,2,4);
plot(D30A0RV);
title('Delay 30 Avg 0')
sgtitle('Comparación aumento delay con referencias variables')

% Comparación con referencias variables en ciclo abierto
figure
subplot(3,1,1);
plot(D10A10RVREF);
yyaxis right;
plot(D10A10RVRPM);
title('Delay 10 Avg 10')
subplot(3,1,2);
plot(D20A20RVREF);
yyaxis right;
plot(D20A20RVRPM);
title('Delay 20 Avg 20')
subplot(3,1,3);
plot(D30A30RVREF);
yyaxis right;
plot(D30A30RVRPM);
title('Delay 30 Avg 30')
sgtitle('Comparación referencias variables y ciclo abierto')

% Comparación en lazo cerrado (control proporcional)
figure
subplot(2,2,1);
plot(D10A10RVREFPROP);
ylim([10 110]);
yyaxis right;
plot(D10A10RVRPMPROP);
ylim([10 110]);
title('Delay 10 Avg 10')
subplot(2,2,2);
plot(D20A20RVREFPROP);
ylim([10 110]);
yyaxis right;
plot(D20A20RVRPMPROP);
ylim([10 110]);
title('Delay 20 Avg 20')
subplot(2,2,3);
plot(D30A30RVREFPROP);
ylim([10 110]);
yyaxis right;
plot(D30A30RVRPMPROP);
ylim([10 110]);
title('Delay 30 Avg 30')
subplot(2,2,4);
plot(D50A50RVREFPROP);
ylim([10 110]);
yyaxis right;
plot(D50A50RVRPMPROP);
ylim([10 110]);
title('Delay 50 Avg 50')
sgtitle('Comparación aumento delay y promediado en ciclo cerrado y referencias variables')

figure
plot(D10A10RVREFPROP);
title('Control proporcional (no a escala)')
yyaxis right;
plot(D10A10RVRPMPROP);