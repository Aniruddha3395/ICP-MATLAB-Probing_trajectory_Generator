function ChkPartSym(src,~)

global check_for_asymmetry;

if (src.Value)
    check_for_asymmetry = true;
else
    check_for_asymmetry = false;
end

end