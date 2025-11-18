FROM robotlocomotion/drake:noble-20250916

RUN apt-get -q update && apt-get -q install -y --no-install-recommends curl nginx-light python3-venv && apt-get remove -y python3-notebook && apt-get autoremove -y && apt-get -q clean

ENV PATH="/opt/drake/bin:${PATH}" \
  PYTHONPATH="/opt/drake/lib/python3.12/site-packages:${PYTHONPATH}"