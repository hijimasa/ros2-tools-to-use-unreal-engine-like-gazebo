import unreal

def import_fbx_and_create_blueprint(fbx_file_path, blueprint_name, wheel_bone_name):
    # Unreal Editor API
    editor_asset_lib = unreal.EditorAssetLibrary()
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()

    # Determine destination paths
    destination_path = "/Game/ImportedFBX"
    fbx_file_name = fbx_file_path.split('/')[-1].replace('.fbx', '')
    skeletal_mesh_path = f"{destination_path}/{fbx_file_name}"
    physics_asset_path = f"{destination_path}/{fbx_file_name}_PhysicsAsset"

    # Check for and delete existing Skeletal Mesh and Physics Asset
    if editor_asset_lib.does_asset_exist(skeletal_mesh_path):
        unreal.log_warning(f"Skeletal Mesh already exists at {skeletal_mesh_path}, deleting...")
        editor_asset_lib.delete_asset(skeletal_mesh_path)

    if editor_asset_lib.does_asset_exist(physics_asset_path):
        unreal.log_warning(f"Physics Asset already exists at {physics_asset_path}, deleting...")
        editor_asset_lib.delete_asset(physics_asset_path)

    # Import FBX File with Skeletal Mesh enabled
    fbx_import_task = unreal.AssetImportTask()
    fbx_import_task.filename = fbx_file_path
    fbx_import_task.destination_path = "/Game/ImportedFBX"
    fbx_import_task.replace_existing = True
    fbx_import_task.automated = True

    # Configure FbxImportUI
    import_options = unreal.FbxImportUI()

    # Explicitly set to import as Skeletal Mesh
    import_options.mesh_type_to_import = unreal.FBXImportType.FBXIT_SKELETAL_MESH
    import_options.original_import_type = unreal.FBXImportType.FBXIT_SKELETAL_MESH
    import_options.import_mesh = True
    import_options.import_as_skeletal = True  # Crucial setting for Skeletal Mesh
    import_options.import_rigid_mesh = True
    import_options.import_materials = True
    import_options.import_textures = True
    import_options.automated_import_should_detect_type = False
    import_options.create_physics_asset = True

    # Skeletal Mesh settings
    import_options.skeletal_mesh_import_data.import_mesh_lo_ds = False
    import_options.skeletal_mesh_import_data.set_editor_property("use_t0_as_ref_pose", True)

    # Static Mesh-specific settings (disabled to avoid default to Static Mesh)
    import_options.static_mesh_import_data.combine_meshes = False
    import_options.static_mesh_import_data.auto_generate_collision = True
    import_options.static_mesh_import_data.one_convex_hull_per_ucx = False

    # Assign options to the task
    fbx_import_task.options = import_options
    fbx_import_task.save = True

    # Execute import task
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([fbx_import_task])

    # Find the imported skeletal mesh
    skeletal_mesh = editor_asset_lib.load_asset(skeletal_mesh_path)

    if not skeletal_mesh:
        unreal.log_error(f"Failed to load Skeletal Mesh from {skeletal_mesh_path}")
        return

    physics_asset = editor_asset_lib.load_asset(physics_asset_path)
    physics_asset_data = editor_asset_lib.find_asset_data(physics_asset_path)

    if not physics_asset or not physics_asset_data:
        unreal.log_error(f"Failed to create PhysicsAsset for {skeletal_mesh.get_name()}")
        return
        
    # Delete existing Blueprint
    blueprint_path = "/Game/" + blueprint_name
    if editor_asset_lib.does_asset_exist(blueprint_path):
        success = editor_asset_lib.delete_asset(blueprint_path)
        if success:
            print(f"Successfully deleted existing Blueprint: {blueprint_path}")
        else:
            unreal.log_error(f"Failed to delete existing Blueprint: {blueprint_path}")
            return
    else:
        print(f"No existing Blueprint found at {blueprint_path}, skipping deletion.")

    # Create Blueprint
    bp_factory = unreal.BlueprintFactory()
    bp_factory.set_editor_property("parent_class", unreal.Actor)
    blueprint = asset_tools.create_asset(blueprint_name, "/Game", unreal.Blueprint, bp_factory)

    blueprint_generated_class = unreal.BlueprintEditorLibrary.generated_class(blueprint)
    blueprint_default_object = blueprint_generated_class.get_default_object()

    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    handle = subsystem.k2_gather_subobject_data_for_blueprint(blueprint)[1]

    skeletal_mesh_handle, fail_reason = subsystem.add_new_subobject(
                                unreal.AddNewSubobjectParams(
                                    parent_handle=handle,
                                    new_class=unreal.SkeletalMeshComponent,
                                    blueprint_context=blueprint
                                    )
                                )

    # Set Skeletal Mesh
    skeletal_mesh_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(skeletal_mesh_handle)
    skeletal_mesh_component = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(skeletal_mesh_data)
    skeletal_mesh_component.set_editor_property("skeletal_mesh", skeletal_mesh)

    # Apply Convex Decomposition Collision
    skeletal_mesh_component.set_all_bodies_simulate_physics(True)
    skeletal_mesh_component.set_simulate_physics(True)
    skeletal_mesh_component.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    skeletal_mesh_component.set_collision_object_type(unreal.CollisionChannel.ECC_PHYSICS_BODY)

    # Add custom C++ component under SkeletalMeshComponent
    custom_component_handle, _ = subsystem.add_new_subobject(
        unreal.AddNewSubobjectParams(
            parent_handle=handle,
            new_class=unreal.RRBaseRobot,
            blueprint_context=blueprint
        )
    )
    # (Optional) Set properties on the custom component
    #custom_component_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(custom_component_handle)
    #if not custom_component_data:
    #    unreal.log_error("component data is None")
    #custom_component = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(custom_component_data)
    #custom_object = unreal.RRBaseRobot()
    #if not custom_component:
    #    unreal.log_error("component is None")
    #custom_component.set_editor_property("property_name", custom_object)

    # Save Blueprint
    unreal.EditorAssetLibrary.save_loaded_asset(blueprint)

    # Place Blueprint in the world at the origin
    bp_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
        blueprint_generated_class,
        location=unreal.Vector(0.0, 0.0, 0.0),  # World origin
        rotation=unreal.Rotator(0.0, 0.0, 0.0)
    )

    print(f"Blueprint {blueprint_name} created and placed in the world origin.")

# Example usage
import_fbx_and_create_blueprint(
    "FBX_PATH",  # Path to the FBX file
    "BP_ROBOT_NAME",      # Name of the Blueprint
    "right_wheel_joint_continuous_bone"                # Name of the wheel bone
)
