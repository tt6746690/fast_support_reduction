function saveMesh(fileName, NV, NF)
  % save NV, NF into fileName
  % saveMesh(fileName, NV, NF)
  %
  % Inputs:
  %  fileName the file to be saved obj
  %  NV  list of vertex positions
  %  NF  list of face indices
  
  fileID = fopen(fileName,'w');
  fprintf(fileID,'v %8.4f %8.4f 0.0000\n', NV');
  fprintf(fileID, 'f %d %d %d\n', NF');
  fclose(fileID);


end     