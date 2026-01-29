import duckdb
import os
import math
import shutil

# --- Configuration ---
INPUT_FILE = "/home/juelg/code/rcs_modern/utn_pick_green_cuboid_v2/2026-01-26_part-0.parquet"
OUTPUT_DIR = "dataset_shards"
TARGET_CHUNK_SIZE_MB = 256  # The optimal size for Parquet

def smart_split():
    # 1. Calculate Target Chunks based on File Size
    if not os.path.exists(INPUT_FILE):
        print(f"Error: {INPUT_FILE} not found.")
        return

    file_size_bytes = os.path.getsize(INPUT_FILE)
    file_size_mb = file_size_bytes / (1024 * 1024)
    target_files = math.ceil(file_size_mb / TARGET_CHUNK_SIZE_MB)

    print(f"File Size:  {file_size_mb:.2f} MB")
    print(f"Target:     {TARGET_CHUNK_SIZE_MB} MB per file")
    print(f"Split Plan: {target_files} files")

    # 2. Get Total Rows via DuckDB
    con = duckdb.connect()
    print("Reading row count (metadata scan)...")
    total_rows = con.sql(f"SELECT count(*) FROM '{INPUT_FILE}'").fetchone()[0]
    
    rows_per_chunk = math.ceil(total_rows / target_files)
    print(f"Total Rows: {total_rows}")
    print(f"Chunk Size: ~{rows_per_chunk} rows")

    # 3. Execute Split
    # We use row_number() // rows_per_chunk to generate a partition key (0, 1, 2...)
    print("Splitting data...")
    con.sql(f"""
        COPY (
            SELECT *, (row_number() OVER () - 1) // {rows_per_chunk} AS chunk_id
            FROM '{INPUT_FILE}'
        ) 
        TO '{OUTPUT_DIR}' 
        (FORMAT PARQUET, PARTITION_BY (chunk_id), COMPRESSION 'SNAPPY', OVERWRITE_OR_IGNORE)
    """)

    # 4. Clean up Folders
    # DuckDB writes to dataset_shards/chunk_id=0/data.parquet
    # We move them to dataset_shards/part-0000.parquet
    print("Finalizing filenames...")
    for item in os.listdir(OUTPUT_DIR):
        if item.startswith("chunk_id="):
            try:
                chunk_num = int(item.split("=")[1])
                src_folder = os.path.join(OUTPUT_DIR, item)
                
                # Find the parquet file inside
                for f in os.listdir(src_folder):
                    if f.endswith(".parquet"):
                        src_file = os.path.join(src_folder, f)
                        dst_file = os.path.join(OUTPUT_DIR, f"part-{chunk_num:04d}.parquet")
                        shutil.move(src_file, dst_file)
                
                # Remove the empty directory
                os.rmdir(src_folder)
            except Exception as e:
                print(f"Skipping {item}: {e}")

    print(f"Done! {target_files} files created in '{OUTPUT_DIR}'")

if __name__ == "__main__":
    smart_split()