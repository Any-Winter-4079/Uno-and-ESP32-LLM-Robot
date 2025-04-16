"""
Wikipedia Summary Fetcher
Searches Wikipedia and returns a plain-text summary of the first matching article.
Features:
- Uses the Wikipedia API for both search and article retrieval
- Handles no-result and error scenarios gracefully
- Returns concise article introductions (extracts)
- Supports configurable maximum summary length
- Easily testable with command-line entry point
"""

import requests

def wiki_search(query, max_summary_length=1000):
    """
    Perform a Wikipedia search and return the summary of the first result.

    Args:
        query: A string representing the search query.
        max_summary_length: Integer defining the maximum number of characters 
                            to return from the summary.

    Returns:
        A string containing either a truncated summary of the article,
        a "no result" message, or an error message.
    """
    base_url = "https://en.wikipedia.org/w/api.php"
    
    # Search for the query
    search_params = {
        "action": "query",
        "format": "json",
        "list": "search",
        "srsearch": query,
        "utf8": 1,
        "srlimit": 1
    }

    try:
        search_response = requests.get(base_url, params=search_params)
        search_response.raise_for_status()
        search_data = search_response.json()

        # Check if any search results were returned
        if not search_data["query"]["search"]:
            return f"\nNo Wikipedia results found for '{query}'.\n"

        #  Fetch summary for the top result
        top_title = search_data["query"]["search"][0]["title"]
        summary_params = {
            "action": "query",
            "format": "json",
            "prop": "extracts",
            "exintro": True,
            "explaintext": True,
            "titles": top_title
        }

        summary_response = requests.get(base_url, params=summary_params)
        summary_response.raise_for_status()
        summary_data = summary_response.json()

        # Parse page dictionary
        page = next(iter(summary_data["query"]["pages"].values()))
        summary = page.get("extract", "No summary available.")
        return f"\n{summary[:max_summary_length]}...\n"

    except requests.RequestException as e:
        return f"\nWikipedia search error: {e}\n"

if __name__ == "__main__":
    query = "Who won the 2023/24 NBA Championship?"
    print(wiki_search(query))
