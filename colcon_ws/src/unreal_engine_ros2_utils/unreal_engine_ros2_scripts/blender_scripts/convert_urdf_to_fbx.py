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

# スムージング設定の適用
collision_objects = []
for obj in bpy.context.scene.objects:
    if obj.type == 'MESH':  # メッシュオブジェクトのみ処理
        # スムージング処理を適用
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.shade_smooth()

        # Auto Smoothを有効化（Meshデータを操作）
        mesh_data = obj.data  # メッシュデータへの参照
        if hasattr(mesh_data, "use_auto_smooth"):  # 属性が存在するか確認
            mesh_data.use_auto_smooth = True
            mesh_data.auto_smooth_angle = 0.523599  # 30度（デフォルト）

        # 衝突メッシュを生成
        #collision_obj = bpy.context.active_object  # 複製されたオブジェクトを取得
        collision_obj = obj.copy()
        collision_obj.data = obj.data.copy()
        collision_obj.name = f"UCX_{obj.name}"  # 衝突メッシュの名前を変更
        collision_objects.append(collision_obj)

        # 衝突メッシュを非表示に設定
        #collision_obj.hide_viewport = True
        #collision_obj.hide_render = True

for collision_obj in collision_objects:
    bpy.context.collection.objects.link(collision_obj)
    
# リストポーズを設定
for obj in bpy.context.scene.objects:
    if obj.type == 'ARMATURE':  # アーマチュアオブジェクトのみ処理
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='POSE')  # ポーズモードに切り替え
        bpy.ops.pose.armature_apply()  # 現在のポーズをリストポーズに適用
        bpy.ops.object.mode_set(mode='OBJECT')  # オブジェクトモードに戻る

# ウェイトペイントを自動生成
for obj in bpy.context.scene.objects:
    if obj.type == 'MESH':  # メッシュオブジェクトのみ処理
        armature = None
        # アーマチュア（スケルトン）を探す
        for modifier in obj.modifiers:
            if modifier.type == 'ARMATURE':
                armature = modifier.object
                break
        
        if armature:
            # メッシュとアーマチュアを選択
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            armature.select_set(True)
            bpy.context.view_layer.objects.active = armature
            
            # ウェイトペイントを自動生成
            bpy.ops.object.parent_set(type='ARMATURE_AUTO')
        else:
            print(f"No armature found for {obj.name}")

# トランスフォームの適用
for obj in bpy.context.scene.objects:
    if obj.type in {'MESH', 'ARMATURE'}:  # メッシュとアーマチュア両方に適用
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# FBX形式でエクスポート
bpy.ops.export_scene.fbx(
    filepath=output_file,
    embed_textures=True,
    path_mode='COPY',
    mesh_smooth_type='FACE',  # スムージングを適用
    add_leaf_bones=False,
    use_tspace=True,
    use_armature_deform_only=True,
)
