function v = OP_ADDNONZEROS_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 89);
  end
  v = vInitialized;
end