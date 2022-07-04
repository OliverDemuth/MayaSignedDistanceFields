print(ligLength('LScH_orig','LScH_dorsalis_ins','myJoint', 'prox_mesh', 'dist_mesh')) # works and result is correct
print(ligLength('LScH_dorsalis_ins','LScH_orig','myJoint', 'prox_mesh', 'dist_mesh')) # works and result is correct

print(ligLength('LScH_orig','LScH_ventralis_ins','myJoint', 'prox_mesh', 'dist_mesh')) # works and result is correct
print(ligLength('LScH_ventralis_ins','LScH_orig','myJoint', 'prox_mesh', 'dist_mesh')) # works and result is correct

print(ligLength('LCoH_dorsalis_orig','LCoH_dorsalis_ins','myJoint', 'prox_mesh', 'dist_mesh')) # works and result is correct
print(ligLength('LCoH_dorsalis_ins','LCoH_dorsalis_orig','myJoint', 'prox_mesh', 'dist_mesh')) # works and result is correct

print(ligLength('LAcH_orig_2','LAcH_ins_2','myJoint', 'prox_mesh', 'dist_mesh')) # works and result is correct
print(ligLength('LAcH_ins_2','LAcH_orig_2','myJoint', 'prox_mesh', 'dist_mesh')) # can't find solution

print(ligLength('LAcH_orig_1','LAcH_ins_1','myJoint', 'prox_mesh', 'dist_mesh')) # crashes/gets stuck in infinite loop after counter 54 for unkwown reasons
print(ligLength('LAcH_ins_1','LAcH_orig_1','myJoint', 'prox_mesh', 'dist_mesh')) # can't find solution
