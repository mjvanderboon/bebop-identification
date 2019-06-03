function [results] = GenerateResults(model)
Aphi     = round(model.Parameters(1).Value,4)
Atheta   = round(model.Parameters(2).Value,4)
Avz      = round(model.Parameters(3).Value,4)
Avpsi    = round(model.Parameters(4).Value,4)
Bphi	= round(model.Parameters(5).Value,4)
Btheta = round(model.Parameters(6).Value,4)
Bvz    = round(model.Parameters(7).Value,4)
Bvpsi  = round(model.Parameters(8).Value,4)
Cphi   = round(model.Parameters(9).Value,4)
Ctheta = round(model.Parameters(10).Value,4)
Cvz    = round(model.Parameters(11).Value,4)
Cvpsi  = round(model.Parameters(12).Value,4)

A = Aphi;
B = Bphi;
C = Cphi;


for i=1:3
    if i == 1
        input = '\phi';
        A = Aphi;
        B = Bphi;
        C = Cphi;
    elseif i==2
        input = '\theta';
        A = Atheta;
        B = Btheta;
        C = Ctheta;
    elseif i == 3
        input = 'v_z';
        A = Avz;
        B = Bvz;
        C = Cvz;
    end
    %hoi = sprintf('\\dot{\\textbf{x}}_%s = \\begin{bmatrix}',input);
    %values = sprintf('%.4f& %.4f & %.4f & %.4f\\\\ %.4f& %.4f & %.4f & %.4f\\\\  %.4f& %.4f & %.4f & %.4f\\\\  %.4f& %.4f & %.4f & %.4f',A');
    %hallo = sprintf('\\end{bmatrix} \\textbf{x}_%s + \\begin{bmatrix}',input);
    %good = sprintf('%.4f \\\\ %.4f \\\\%.4f \\\\%.4f\\end{bmatrix} %s_c',B,input);
    %strcat(hoi, values,hallo,good)
    
    good = sprintf('%s = \\begin{bmatrix} %.4f & %.4f & %.4f & %.4f\\end{bmatrix} \\textbf{x}_%s',input,C,input)      
end
end