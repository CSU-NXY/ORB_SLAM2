"""
目前先把pose加入紧耦合中看看效果
"""
import numpy as np

for seq in ['09', '10']:
    file_path = 'all_dropout_uncert_revised/{}.txt'.format(seq)
    transformationMatrix = np.loadtxt(file_path).reshape(-1, 3, 4)
    transformationMatrix = np.concatenate((transformationMatrix, np.tile(np.array([0,0,0,1]), (transformationMatrix.shape[0], 1, 1))), axis=1).astype(np.float32)

    # T12 = T10 @ T02
    relativeMatrix = np.array([np.linalg.inv(transformationMatrix[i]) @ transformationMatrix[i+1] for i in range(len(transformationMatrix)-1)])

    translation = relativeMatrix[:, :3, 3]

    print('Saving to all_dropout_uncert_revised_translation/{}.txt'.format(seq))
    np.savetxt('all_dropout_uncert_revised_translation/{}.txt'.format(seq), translation)

