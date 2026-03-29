import { useState, useRef } from 'react';
import { geocode } from '../utils/routing';

export default function DestinationSearch({ onSelect, disabled }) {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const debounceRef = useRef(null);

  function handleChange(e) {
    const val = e.target.value;
    setQuery(val);
    setResults([]);
    setError(null);

    clearTimeout(debounceRef.current);
    if (val.trim().length < 3) return;

    debounceRef.current = setTimeout(async () => {
      setLoading(true);
      try {
        const res = await geocode(val);
        setResults(res);
      } catch {
        setError('Search failed. Check your connection.');
      } finally {
        setLoading(false);
      }
    }, 600);
  }

  function handleSelect(result) {
    setQuery(result.display_name);
    setResults([]);
    onSelect(result);
  }

  return (
    <div className="search-container" role="search">
      <input
        className="search-input"
        type="text"
        placeholder="Where do you want to go?"
        value={query}
        onChange={handleChange}
        disabled={disabled}
        aria-label="Destination search"
        autoComplete="off"
      />

      {loading && <p className="search-hint">Searching…</p>}
      {error && <p className="search-hint error">{error}</p>}

      {results.length > 0 && (
        <ul className="search-results" role="listbox" aria-label="Search results">
          {results.map((r, i) => (
            <li
              key={i}
              role="option"
              className="search-result-item"
              onClick={() => handleSelect(r)}
              onKeyDown={(e) => e.key === 'Enter' && handleSelect(r)}
              tabIndex={0}
            >
              {r.display_name}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}