import bpy
import sys
sys.path.append('/home/ue4/.local/lib/python3.11/site-packages')
print(sys.path)

# コマンドライン引数からファイルパスを取得
argv = sys.argv
argv = argv[argv.index("--") + 1:]  # "--"以降の引数を取得
input_file = argv[0]
output_file = argv[1]

# ZIPファイルのパスを指定
addon_zip_path = '/tmp/urdf_importer/urdf_importer_addon.zip'

# アドオンのインストール
bpy.ops.preferences.addon_install(filepath=addon_zip_path)

# アドオンを有効化
bpy.ops.preferences.addon_enable(module='urdf_importer_addon')

# 既存のオブジェクトを削除
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# URDFファイルをインポート
bpy.ops.import_scene.urdf(filepath=input_file)

# FBX形式でエクスポート
bpy.ops.export_scene.fbx(filepath=output_file, embed_textures=True, path_mode='COPY')
