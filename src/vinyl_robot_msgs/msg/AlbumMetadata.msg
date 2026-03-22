# Published by record_identifier when a record is identified
# Subscribed by web_interface for display

string artist
string album_title
string catalog_number
uint16 year
string genre
string[] tracklist
string cover_art_url

# How the record was identified
string identification_method    # "visual", "audio_fingerprint", "manual", "cached"
float32 confidence              # 0.0 – 1.0

# Source identifiers (for caching / repeat lookup avoidance)
string discogs_release_id
string acrcloud_track_id
